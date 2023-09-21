from raya.skills import RayaSkill, RayaSkillHandler
from raya.controllers import MotionController
from raya.application_base import RayaApplicationBase
import asyncio
from .constants import *
import time



from skills.approach_to_tags import SkillApproachToTags


class SkillAttachToCartError(Exception): pass
class SkillAttachToCartErrorGripper(SkillAttachToCartError): pass


class SkillAttachToCart(RayaSkill):


    DEFAULT_SETUP_ARGS = {
            'distance_before_attach': DEFAULT_FINAL_APPROACH_DISTANCE,
            'distance_first_approach':DEFAULT_FIRST_APPROACH_DISTANCE,
            'max_angle_step': DEFAULT_MAX_ANGLE_STEP,
            'timeout' : FULL_APP_TIMEOUT,
            }
    
    REQUIRED_SETUP_ARGS = {
        'tags_size'
    }

    DEFAULT_EXECUTE_ARGS = {
        'turn_clockwise': True,
        'attach': True
    }

    REQUIRED_EXECUTE_ARGS = {
        'identifier',
    }
    

    async def calculate_distance_parameters(self):
        if (self.dl > self.dr):
            self.sign = -1
        else:
            self.sign = 1

        self.delta = self.dl - self.dr
        if (self.delta == 0):
            self.normalized_delta = 0
        else:
            self.normalized_delta = abs(self.delta/(self.dl + self.dr))
        
    async def calculate_velocity(self):
        ### calculate velocity with respect to distance from cart
        self.normalized_delta_position = \
            1-((CART_MAX_DISTANCE - self.SRF)/CART_MAX_DISTANCE)
        
        self.linear_velocity = \
            MAX_LINEAR_MOVING_VELOCITY * self.normalized_delta_position
        if self.linear_velocity < MIN_LINEAR_MOVING_VELOCITY:
            self.linear_velocity = MIN_LINEAR_MOVING_VELOCITY
        elif self.linear_velocity > MAX_LINEAR_MOVING_VELOCITY:
            self.linear_velocity = MAX_LINEAR_MOVING_VELOCITY

    async def sensor_noise_identifier(self):
        if self.SRF > SRF_NOISE_VERIFICATION_DISTANCE and\
            (self.dl > IR_SENSOR_MAX_VALUE_VERIFICATION or\
             self.dr > IR_SENSOR_MAX_VALUE_VERIFICATION):
            self.log.warn('IR sensor recive too much noise')
            self.state = 'finish'
            self.abort(*ERROR_SENSOR_NOISE)

    async def state_classifier(self):
        ### change to parameters
        if (abs(self.delta)>ROTATING_DELTA_MIN and\
             self.normalized_delta > NORMALIZED_DELTA_MAX):

            self.state = 'rotating'
            return True

        elif ((self.dl > ATACHING_DISTANCE_MIN and\
              self.dr > ATACHING_DISTANCE_MIN) or\
                  (self.dl>ATACHING_DISTANCE_MAX and\
                    self.dr>ATACHING_DISTANCE_MAX and\
                          self.normalized_delta < NORMALIZED_DELTA_MIN)):
            if self.attach:
                self.state = 'attaching'
            else:
                self.state = 'detaching'
            return True
        
        else:
            self.state = 'moving'
            return True
    async def timer_update(self):
        self.timer = time.time() - self.start_time

    async def adjust_angle(self):

        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()

        await self.motion.rotate(
            angle= ROTATING_CONST * self.normalized_delta,
            angular_speed = self.sign * ROTATING_ANGULAR_SPEED,
            enable_obstacles=False,
            wait=True)
            
        self.log.info("finish rotate")

    async def gripper_state_classifier(self):

        verification_max_position = GRIPPER_ATTACHED_POSITION + \
        GRIPPER_ATTACHED_THRESHOLD
        verification_min_position = GRIPPER_ATTACHED_POSITION - \
        GRIPPER_ATTACHED_THRESHOLD

        ### TODO uncomment when real cart jig is ready

        # if (self.gripper_state['pressure_reached'] == False and\
            # self.gripper_state['final_position']<verification_max_position and\
            # self.gripper_state['final_position']>verification_min_position):
        if (self.gripper_state['final_position']<verification_max_position and\
            self.gripper_state['final_position']>verification_min_position):                
                self.gripper_state['cart_attached'] = True
        else:
            self.gripper_state['cart_attached'] = False
            self.abort(*ERROR_GRIPPER_ATTACHMENT_FAILED)

    async def cart_attachment_verification(self):
        self.log.info('run cart_attachment_verification')

        verification_SRF_value=self.SRF
        await self.move_linear(sign = 1,wait=True)

        # while (self.motion.is_moving()):
        await self.read_sensor_values()
        await self.calculate_distance_parameters()
        distance_change = abs(verification_SRF_value - self.SRF)

        if distance_change > VERIFICATION_DISTANCE:
            self.gripper_state['cart_attached'] = False
            self.state = 'finish'

            # await asyncio.sleep(0.2)
        cart_attached = self.gripper_state['cart_attached']
        self.state = 'finish'
        if not cart_attached:
            self.log.error('cart verification failed, '
                    f'distance before verification {verification_SRF_value}, '
                    f'distance after verification{self.SRF}')
            self.abort (*ERROR_CART_NOT_ATTACHED)
        else:
            self.log.info(f'cart verification succseeded')
        
            
        
        
    async def attach(self):
        self.log.info("stop moving, start attaching")

        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()
        try:
            gripper_result = await self.arms.specific_robot_command(
                            name='cart/execute',
                            parameters={
                                    'gripper':'cart',
                                    'goal':GRIPPER_CLOSE_POSITION,
                                    'velocity':0.5,
                                    'pressure':GRIPPER_CLOSE_PRESSURE_CONST,
                                    'timeout':10.0
                                }, 
                            wait=True,
                        )
        except Exception as error:
                self.log.error(f'gripper fail error is: {error}'
                                F'error type: {type(error)}')
                self.abort(*ERROR_GRIPPER_ATTACHMENT_FAILED)
                self.state = 'finish'

        await self.gripper_feedback_cb(gripper_result)
        await self.gripper_state_classifier()
        cart_attached = self.gripper_state['cart_attached']
        self.log.info(f'gripper attachment feedback is: {cart_attached}')

        if cart_attached:
            self.state = 'attach_verification'
            await self.cart_attachment_verification()
        else:
            self.state = 'finish'

    async def detach_cart(self):
        await self.motion.set_velocity(x_velocity=self.linear_velocity,y_velocity=0.0,angular_velocity=0.0,duration=REVERSE_DISTANCE/self.linear_velocity,wait=True)
        sign = 1 if self.execute_args['turn_clockwise'] else -1
        await self.motion.rotate(angle=PRE_ATTACH_ANGLE_ROTATION,angular_speed=PRE_ATTACH_RUTATION_SPEED*sign,wait=True)
        try:
            gripper_result = await self.arms.specific_robot_command(
                            name='cart/execute',
                            parameters={
                                    'gripper':'cart',
                                    'goal':GRIPPER_OPEN_POSITION,
                                    'velocity':0.5,
                                    'pressure':GRIPPER_OPEN_PRESSURE_CONST,
                                    'timeout':10.0
                                },
                            wait=True,
                        )
        except Exception as error:
                self.log.error(f'gripper fail error is: {error}'
                                F'error type: {type(error)}')
                self.abort(*ERROR_GRIPPER_ATTACHMENT_FAILED)
                self.state = 'finish'

        await self.gripper_feedback_cb(gripper_result)
        await self.gripper_state_classifier()
        cart_attached = self.gripper_state['cart_attached']
        self.log.info(f'gripper attachment feedback is: {cart_attached}')

        self.state = 'finish'

    async def gripper_feedback_cb(self, gripper_result):
        self.gripper_state['final_position'] = gripper_result['final_position']
        self.gripper_state['final_pressure'] = gripper_result['final_pressure']
        self.gripper_state['position_reached'] = gripper_result['position_reached']
        self.gripper_state['pressure_reached'] = gripper_result['pressure_reached']
        self.gripper_state['success'] = gripper_result['success']
        self.gripper_state['timeout_reached'] = gripper_result['timeout_reached']
        self.log.info(gripper_result)



    async def move_linear(self, sign, wait = False):
        ### TODO try axcept
        await self.calculate_velocity()
        try:
            await self.motion.set_velocity(
                        x_velocity= sign * self.linear_velocity,
                        y_velocity=0.0,
                        angular_velocity=0.0,
                        duration=2.0,
                        enable_obstacles=False,
                        wait = wait,
                    )
        except Exception as error:
            self.log.error(f'linear movement failed, error: {error}')
            self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)
        if sign == -1:
            await self.cart_distance_verification()

    async def timeout_verification (self):
        if self.timer > self.timeout:
            self.log.error(f'timeout reached: {self.timeout} sec')
            self.abort(*ERROR_TIMEOUT_REACHED)
            self.state = 'finish'

    async def cart_max_distance_verification (self):
        if self.SRF > CART_MAX_DISTANCE:
            self.log.error(f'cart is too far, distance: {self.SRF} cm')
            self.abort(*ERROR_CART_NOT_ACCESSABLE)
            self.state = 'finish'

    async def cart_distance_verification (self):
        if self.last_SRF < self.SRF:
                self.pushing_index += 1
        
        if self.pushing_index > POSITION_VERIFICATION_MAX_INDEX:
            self.log.error('failed to attach, cart pushed')
            self.abort(*ERROR_CART_NOT_GETTING_CLOSER)
            self.state = 'finish'



            
    async def pre_loop_actions(self):
        ### move gripper to pre-grab position
        self.pre_loop_finish = True
        try:
            await self.arms.specific_robot_command(
                name='cart/execute',
                parameters={
                        'gripper':'cart',
                        'goal':GRIPPER_OPEN_POSITION,
                        'velocity':0.5,
                        'pressure':GRIPPER_OPEN_PRESSURE_CONST,
                        'timeout':10.0
                    }, 
                wait=True,
            )

        except Exception as error:
                self.log.error(f'gripper error: {error}')
                self.abort(*ERROR_GRIPPER_FAILED)
                self.state = 'finish'
        # run approach to tag
        try:
            await self.skill_apr2tags.execute_setup(
                setup_args={
                        'working_cameras': CAMERAS_TO_USE,
                        'tags_size': self.tags_size,
                    },
            )
            approach_result = await self.skill_apr2tags.execute_main(
                    execute_args={
                        'identifier': self.identifier,
                        'distance_to_goal': self.distance_before_attach,
                        'max_angle_error_allowed': MAX_ANGLE_ERROR_ALLOWED,
                        'max_y_error_allowed': MAX_Y_ERROR_ALLOWED
                    },
                    callback_feedback=self.cb_approach_feedback
                )
            await self.skill_apr2tags.execute_finish(
                wait=False
            )
            self.log.debug(approach_result)

        except Exception as error:
            self.log.error ('approach failed, Exception type:'
                            f'{type(error)}, Exception: {error}')
            self.abort(*ERROR_APPROACH_FAILED)
            self.state = 'finish'
            self.pre_loop_finish = False
            
        
        # rotate gary 180 degrees
        try:
            await self.motion.rotate(
                angle = PRE_ATTACH_ANGLE_ROTATION,
                angular_speed= PRE_ATTACH_RUTATION_SPEED,
                enable_obstacles=False,
                wait=True)
        except Exception as error:
            self.log.error(f'rotation movement failed, error: {error}')
            self.abort(*ERROR_ROTATION_MOVEMENT_FAILED)
            self.pre_loop_finish = False
        
        
        self.dl=self.sensors.get_sensor_value('cart_sensor')\
            [f'{IR_SENSOR_ID_LEFT}']
        self.dr=self.sensors.get_sensor_value('cart_sensor')\
            [f'{IR_SENSOR_ID_RIGHT}']
        
        ### small angle adjusment
        # await self.read_sensor_values()
        # await self.calculate_distance_parameters()
        # await self.adjust_angle()


        return self.pre_loop_finish
            
    async def read_sensor_values(self):
        self.last_dl = self.dl
        self.last_dr = self.dr
        self.last_SRF = self.SRF

        self.dl=self.sensors.get_sensor_value('cart_sensor')\
            [f'{IR_SENSOR_ID_LEFT}']
        self.dr=self.sensors.get_sensor_value('cart_sensor')\
            [f'{IR_SENSOR_ID_RIGHT}']
        self.SRF=self.sensors.get_sensor_value('srf')\
            [f'{SRF_SENSOR_ID_CENTER}'] * 100
        await self.sensor_noise_identifier()


    async def setup(self):
        self.skill_apr2tags:RayaSkillHandler = \
                self.register_skill(SkillApproachToTags)

        self.arms = await self.enable_controller('arms')
        self.sensors = await self.enable_controller('sensors')
        self.motion = await self.enable_controller('motion')

        # calibrate cart gripper
        try:
            await self.arms.specific_robot_command(
                name='cart/calibrate',
                parameters={'hand':'cart'}, 
                wait=True
            )
        except Exception as error:
                
                print ('error calibrate')
                self.log.error(f'calibration error: {error}')
                self.abort(*ERROR_GRIPPER_FAILED)
                self.state = 'finish'
            
        # declare parametrs from setup args
        self.distance_before_attach = self.setup_args['distance_before_attach']
        self.distance_first_approach = self.setup_args['distance_first_approach']
        self.max_angle_step = self.setup_args['max_angle_step']
        self.tags_size = self.setup_args['tags_size']
        self.identifier = self.setup_args['identifier']
        self.timeout = self.setup_args['timeout']

        #declare variables
        self.linear_velocity = 0
        self.SRF = 0
        self.sign = 1
        self.state = 'idle'
        self.normalized_delta = 0
        self.pushing_index = 0
        

        self.gripper_state = {'final_position': 0.0,
                            'final_pressure': 0.0,
                            'position_reached': False,
                            'pressure_reached': False,
                            'success': False,
                            'timeout_reached': False,
                            'cart_attached': False}
        


    async def main(self):
        ### approach state

        self.log.info('SkillAttachToCart.main')

        await self.pre_loop_actions() ### approach and rotate 

        self.start_time = time.time()
        self.timer = self.start_time
        await self.read_sensor_values()
        
        while (True): ### main loop
    
            await self.read_sensor_values()

            await self.timer_update()

            await self.timeout_verification()

            await self.calculate_distance_parameters()

            await self.cart_max_distance_verification()

            if self.state == 'finish':
                cart_attached = self.gripper_state['cart_attached']
                self.log.info('application finished, cart attachment is: '\
                            f'{cart_attached}')
                break

            else:
                await self.state_classifier()
            if self.state == 'moving':
                await self.move_linear(sign = -1)
                
            elif (self.state == 'rotating'):
                self.pushing_index = 0
                await self.adjust_angle()

            elif self.state == 'attaching': ##attach and verify cart attachment
                await self.attach() 
            elif self.state == 'detaching':
                await self.detach_cart()
            
                
            await asyncio.sleep(0.2)



    async def finish(self):
        self.skill_apr2tags.wait_finish()
        self.log.info('SkillAttachToCart.finish')

    async def cb_approach_feedback(self, feedback):
        self.log.debug(feedback)