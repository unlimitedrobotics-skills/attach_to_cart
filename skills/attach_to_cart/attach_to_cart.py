from raya.skills import RayaSkill, RayaSkillHandler
from raya.controllers import MotionController
from raya.application_base import RayaApplicationBase
import asyncio
from .constants import *
import time



from skills.approach_to_tags import SkillApproachToTags


class SkillAttachToCart(RayaSkill):


    DEFAULT_SETUP_ARGS = {
            'distance_before_attach': DEFAULT_FINAL_APPROACH_DISTANCE,
            'distance_first_approach':DEFAULT_FIRST_APPROACH_DISTANCE,
            'max_angle_step': DEFAULT_MAX_ANGLE_STEP,
            'timeout' : FULL_APP_TIMEOUT,

            }
    REQUIRED_SETUP_ARGS = {
        'identifier',
        'tags_size'
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
            self.state = 'attaching'
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
        if (self.gripper_state['pressure_reached'] == True and\
            self.gripper_state['position_reached'] == False):
                
                self.gripper_state['cart_attached'] = True
        else:
            self.gripper_state['cart_attached'] = True
            self.abort(ERROR_GRIPPER_ATTACHMENT_FAILED)

    async def cart_attachment_verification(self):
        self.log.info('run cart_attachment_verification')

        verification_SRF_value=self.SRF
        await self.move_linear(sign = 1)

        while (self.motion.is_moving()):
            await self.read_sensor_values()
            await self.calculate_distance_parameters()

            if abs(verification_SRF_value - self.SRF) > VERIFICATION_DISTANCE:
                self.gripper_state['cart_attached'] = False
                self.state = 'finish'
                break

            await asyncio.sleep(0.2)
        cart_attached = self.gripper_state['cart_attached']
        self.log.info(f'cart verification is: {cart_attached}')
        if cart_attached:
            self.state = 'finish'
        else:
            self.abort (ERROR_CART_NOT_ATTACHED)
        
        
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
            await self.gripper_feedback_cb(gripper_result)
            await self.gripper_state_classifier()
            cart_attached = self.gripper_state['cart_attached']
            self.log.info(f'gripper attachment feedback is: {cart_attached}')

            if cart_attached:
                self.state = 'attach_verification'
                await self.cart_attachment_verification()
            else:
                self.state = 'finish'
            

        except Exception as error:

            self.log.error(
                f'gripper attachment failed, Exception type: '
                f'{type(error)}, Exception: {error}')

            pass

    async def gripper_feedback_cb(self, gripper_result):
        self.gripper_state['final_position'] = gripper_result['final_position']
        self.gripper_state['final_pressure'] = gripper_result['final_pressure']
        self.gripper_state['position_reached'] = gripper_result['position_reached']
        self.gripper_state['pressure_reached'] = gripper_result['pressure_reached']
        self.gripper_state['success'] = gripper_result['success']
        self.gripper_state['timeout_reached'] = gripper_result['timeout_reached']


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
            ERROR = [*ERROR_LINEAR_MOVEMENT_FAILED]
            ERROR[1] += f'{type(error)}, Exception: {error}'
            self.log.abort(*ERROR)
        if sign == -1:
            await self.cart_distance_verification()

    async def timeout_verification (self):
        if self.timer > self.timeout:
            ERROR = [*ERROR_TIMEOUT_REACHED]
            ERROR[1] +=  f'timeout: {self.timeout}'
            self.abort(*ERROR)
            self.state = 'finish'

    async def cart_max_distance_verification (self):
        if self.SRF > CART_MAX_DISTANCE:
            ERROR = [*ERROR_CART_NOT_ACCESSABLE]
            ERROR[1] +=  f'distance: {self.SRF}'
            self.abort(*ERROR)
            self.state = 'finish'

    async def cart_distance_verification (self):
        if self.last_SRF < self.SRF:
                self.pushing_index += 1
        
        if self.pushing_index > POSITION_VERIFICATION_MAX_INDEX:
            self.log.error('error, cart is not getting closer'
                           f'for {self.pushing_index} max readings')
            self.state = 'finish'



            
    async def pre_loop_actions(self):
        ### move gripper to pre-grab position
        self.pre_loop_finish = True
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
                            wait=False,
                        )
            self.log.info(f'gripper_result: {type(gripper_result)}')
            await self.gripper_feedback_cb(gripper_result)
            print(gripper_result)

        except Exception as error:
                self.log.error(
                    f'gripper open to pre-grab position failed, Exception '
                    f'type: {type(error)}, Exception: {error}')
                pass
        # run approach to tag
        try:
            chest_approach_result = await self.skill_apr2tags.run(
                setup_args={
                        'working_cameras': ['nav_bottom', 'nav_top'],
                        'identifier': self.identifier,
                        'tags_size': self.tags_size,
                    },
                    execute_args={
                        'distance_to_goal': self.distance_before_attach,
                        'max_angle_error_allowed': MAX_ANGLE_ERROR_ALLOWED,
                        'max_y_error_allowed': MAX_Y_ERROR_ALLOWED
                    },
                    callback_feedback=self.cb_approach_feedback
                )
            self.log.debug(chest_approach_result)

        except Exception as error:
            ERROR = [*ERROR_APPROACH_FAILED]
            ERROR[1] +=  f'{type(error)}, Exception: {error}'
            self.abort(*ERROR)
            self.state = 'finish'
            self.pre_loop_finish = False
            
        
        # rotate gary 180 degrees
        try:
            await self.motion.rotate(
                angle = PRE_ATTACH_ANGLE,
                angular_speed= PRE_ATTACH_RUTATION_SPEED,
                enable_obstacles=False,
                wait=True)
        except Exception as error:
            self.log.error(
                f'180 degree rotation failed, Exception type: '
                f'{type(error)}, Exception: {error}')
            self.pre_loop_finish = False
            raise error
        
        # small angle adjusment
        self.dl=self.sensors.get_sensor_value('cart_sensor')\
            [f'{IR_SENSOR_ID_LEFT}']
        self.dr=self.sensors.get_sensor_value('cart_sensor')\
            [f'{IR_SENSOR_ID_RIGHT}']
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


    async def setup(self):
        self.skill_apr2tags:RayaSkillHandler = \
                self.register_skill(SkillApproachToTags)

        self.arms = await self.enable_controller('arms')
        self.sensors = await self.enable_controller('sensors')
        self.motion = await self.enable_controller('motion')

        ## calibrate cart gripper
        await self.arms.specific_robot_command(
            name='cart/calibrate',
            parameters={'hand':'cart'}, 
            wait=False
    )
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

            
                
            await asyncio.sleep(0.2)



    async def finish(self):
        self.log.info('SkillAttachToCart.finish')

    async def cb_approach_feedback(self, feedback):
        self.log.debug(feedback)