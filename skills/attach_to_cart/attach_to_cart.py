from raya.skills import RayaSkill, RayaSkillHandler
from raya.controllers import MotionController
from raya.application_base import RayaApplicationBase
import asyncio
from .constants import *
from raya.controllers.navigation_controller import POSITION_UNIT, ANGLE_UNIT

from skills.approach_to_tags import SkillApproachToTags
import math
import time



class SkillAttachToCart(RayaSkill):


    DEFAULT_SETUP_ARGS = {
            # 'distance_before_attach': 0.5,
            # 'distance_first_approach':1.0,
            # 'max_angle_step': 15.0
        'timeout' : FULL_APP_TIMEOUT,
            }
    REQUIRED_SETUP_ARGS = {
        'actual_desired_position'
    }
    

    async def calculate_distance_parameters(self):
        ## calculate the distance of each dl and dr (distances) and calculate the angle of oreitattion related to the normal to the cart
        if (self.dl > self.dr):
            self.sign = -1
        else:
            self.sign = 1

        self.delta = self.dl - self.dr
        self.angle  = math.atan2(self.delta,DISTANCE_BETWEEN_SRF_SENSORS)/math.pi * 180
        self.last_average_distance = self.average_distance
        self.log.info(f'left:{self.dl}, right:{self.dr}, angle: {self.angle}')

    async def pushing_cart_identifier(self):
        if self.last_average_distance > self.last_average_distance:
            self.pushing_index += 1
            self.log.warn(f'cart seems to be pushed by gary, index: {self.pushing_index}')
        if self.pushing_index > MAX_PUSHING_INDEX:
            self.log.error(f'cart pushed by gary {self.pushing_index} times')
            self.abort(*ERROR_CART_NOT_GETTING_CLOSER)


    async def state_classifier(self):
        ### change to parameters
        ## rotating state
        ## If self.dl is less then rotating disance or right
        # and also the sum is less then average
        # and self.angle is above rotating..

        self.log.debug(f'current state: {self.state}')

        if (self.state == 'attach_verification'):
            return True

        elif (self.state == 'finish'):
            return True
        
        elif ((self.dl<ROTATING_DISTANCE or self.dr<ROTATING_DISTANCE) and\
             (self.dl+self.dr)/2 < ROTATING_DISTANCE_AV and\
                  abs(self.angle) > ROTATING_ANGLE_MIN):

            self.state = 'rotating'
            return True
        
        ## If the sensor distance is low then min every thing ok you can close
        ## If the distance is lower then max size and also the orientation angle is low - close ok
        elif ((self.dl < ATACHING_DISTANCE_MIN and\
              self.dr < ATACHING_DISTANCE_MIN) or\
                  (self.dl<ATACHING_DISTANCE_MAX and\
                    self.dr<ATACHING_DISTANCE_MAX and\
                          abs(self.angle)<ATACHING_ANGLE_MAX)):
            self.state = 'attaching'
            return True
        
        else:
            self.state = 'moving'
            return True
        
    async def adjust_angle(self):
        ## Control law for minimzing the angle between the cart to the robot
        if (self.angle > MAX_ANGLE_STEP):
            self.angle = MAX_ANGLE_STEP
        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()

        await self.motion.rotate(
            angle= max(abs(self.angle) * ROTATION_KP, MIN_ROTATION_ANGLE_STEP),
            angular_speed= self.sign * ROTATING_ANGULAR_SPEED,
            enable_obstacles=False,
            wait=True)
            
        self.log.info("finish rotate")

    async def gripper_state_classifier(self):
        ### TODO add position value check
        # Check if the position (0 or 1) and pressure were reached
        if (self.gripper_state['pressure_reached'] == True and \
            self.gripper_state['position_reached'] == False):

            # If the pressure was reached but the position wasnt reached, that
            # means the adapter touched something. Check if the adapter is close
            # to the actual desired position and mark the cart as attached
            if self.gripper_state['close_to_actual_position'] == True:
                self.gripper_state['cart_attached'] = True

            # If its not, try to attach again
            else:
                await self.send_feedback('Actual desired position not reached. Attaching again...')
                self.state = 'attaching'

        else:
            self.gripper_state['cart_attached'] = False
            self.state = 'finish'

    async def cart_attachment_verification(self):
        self.log.info('run cart_attachment_verification')
        verification_dl=self.dl
        verification_dr=self.dr
        verification_delta = verification_dl - verification_dr
        verification_angle  = abs(math.atan2(verification_delta,DISTANCE_BETWEEN_SRF_SENSORS)/math.pi *180)

        await self.motion.set_velocity(
                x_velocity = VERIFICATION_VELOCITY,
                y_velocity = 0.0,
                angular_velocity=0.0,
                duration=2.0,
                enable_obstacles=False,
                wait=False, 
                )
        
        while (self.motion.is_moving()):
            dl=self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_RIGHT] * 100
            dr=self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_LEFT] * 100
            delta = dl - dr
            angle  = abs(math.tan(delta/DISTANCE_BETWEEN_SRF_SENSORS)/math.pi * 180)
            await asyncio.sleep(0.2)

            if dl < VERIFICATION_DISTANCE or dr < VERIFICATION_DISTANCE:
                self.log.info('finish cart attach verification')
                self.gripper_state['cart_attached'] = True
            else:
                self.gripper_state['cart_attached'] = False
        
        self.state = 'finish'

    async def vibrate(self):
        try:
            await self.motion.set_velocity(
                    x_velocity = VERIFICATION_VELOCITY,
                    y_velocity = 0.0,
                    angular_velocity=0.0,
                    duration=0.5,
                    enable_obstacles=False,
                    wait=True, 
                    )
            await self.motion.set_velocity(
                    x_velocity = -VERIFICATION_VELOCITY,
                    y_velocity = 0.0,
                    angular_velocity=0.0,
                    duration=0.3,
                    enable_obstacles=False,
                    wait=True, 
                    )
        except Exception as error:
            self.log.error(f'linear movement failed, error: {error}')
            self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)

    async def attach(self):
        self.log.info("stop moving, start attaching")

        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()
        try:

            while(True):
                await self.sleep(1.0)
                if(self.gripper_state['attempts'] > MAX_ATTEMPTS):
                    self.state = "finish"
                    break

                gripper_result = await self.arms.specific_robot_command(
                                        name='cart/execute',
                                        parameters={
                                                'gripper':'cart',
                                                'goal':GRIPPER_CLOSE_POSITION,
                                                'velocity':GRIPPER_VELOCITY,
                                                'pressure':GRIPPER_CLOSE_PRESSURE_CONST,
                                                'timeout':25.0
                                            }, 
                                        wait=True,
                                    )
                
                self.log.debug(f'gripper result: {gripper_result}')

                await self.gripper_feedback_cb(gripper_result)
                await self.gripper_state_classifier()
    
                cart_attached = self.gripper_state['cart_attached']
                if cart_attached:
                    self.state = 'attach_verification'
                    break
                self.gripper_state['attempts']+=1
                if self.gripper_state['attempts'] > ATTEMPTS_BEFORE_VIBRATION:
                   await self.vibrate()


            await self.send_feedback(gripper_result)
            await self.send_feedback({'cart_attached_success' : cart_attached})
            
            

        # except RayaApplicationNotRegistered:
        #     pass
        except Exception as error:
                self.log.error(f'gripper fail error is: {error}'
                                F'error type: {type(error)}')
                self.abort(*ERROR_GRIPPER_ATTACHMENT_FAILED)
                self.state = 'finish'

    async def gripper_feedback_cb(self, gripper_result):
        ## add number of attemps
        self.gripper_state['final_position'] =  gripper_result['final_position']
        self.gripper_state['final_pressure'] = gripper_result['final_pressure']
        self.gripper_state['position_reached'] = gripper_result['position_reached']
        self.gripper_state['pressure_reached'] = gripper_result['pressure_reached']
        self.gripper_state['success'] = gripper_result['success']
        self.gripper_state['timeout_reached'] = gripper_result['timeout_reached']
        if abs(gripper_result['final_position'] - self.setup_args['actual_desired_position']) < POSITION_ERROR_MARGIN: 
            self.gripper_state['close_to_actual_position'] = True
        else:
            self.log.info(f'Attemps,{self.gripper_state["attempts"]}, final_position {gripper_result["final_position"]}')



    async def move_backwared(self):
        ### TODO try axcept
        kp = VELOCITY_KP
        cmd_velocity = kp*self.average_distance
        self.log.info(f'cmd_velocity {cmd_velocity}')

        if abs(cmd_velocity) > MAX_MOVING_VELOCITY:
            cmd_velocity = MAX_MOVING_VELOCITY
        try:
            await self.motion.set_velocity(
                        x_velocity= -1 * cmd_velocity,
                        y_velocity=0.0,
                        angular_velocity=0.0,
                        duration=2.0,
                        enable_obstacles=False,
                        wait=False,
                    )
        except Exception as error:
            self.log.error(f'linear movement failed, error: {error}')
            self.abort(*ERROR_LINEAR_MOVEMENT_FAILED)
        
    async def _timer_update(self):
        self.timer = time.time() - self.start_time

    async def _timeout_verification (self):
        if self.timer > self.timeout:
            self.log.error(f'timeout reached: {self.timeout} sec')
            self.abort(*ERROR_TIMEOUT_REACHED)
            self.state = 'finish'

    async def pre_loop_actions(self):
        ## set the stats to diffault
        await self.set_to_diffualt()
        ### move gripper to pre-grab position

        self.pre_loop_finish = True
        try:
            gripper_result = await self.arms.specific_robot_command(
                                                    name='cart/execute',
                                                    parameters={
                                                            'gripper':'cart',
                                                            'goal':GRIPPER_OPEN_POSITION,
                                                            'velocity':GRIPPER_VELOCITY,
                                                            'pressure':GRIPPER_OPEN_PRESSURE_CONST,
                                                            'timeout':25.0
                                                        }, 
                                                    wait=True,
                                                )
            
            self.log.debug(f'gripper result: {gripper_result}')

        # except RayaApplicationNotRegistered:
        
        #     pass
            
        except Exception as error:
            self.log.error(
                f'gripper open to pre-grab position failed, Exception type: '
                f'{type(error)}, Exception: {error}')
            self.abort(*ERROR_GRIPPER_FAILED)
        
        ### rotate 180 degree
        await self.rotation_180()
        await self.read_srf_values()
        await self.calculate_distance_parameters()
        await self._cart_max_distance_verification()
        await self.major_angle_identification()



       
        return self.pre_loop_finish

    # async def get_orientation(self):
        # self.orientation = await self.nav.get_position(pos_unit=POSITION_UNIT.METERS, ang_unit=ANGLE_UNIT.DEGREES)
    async def rotation_180(self):
        self.log.info('Rotating 180 degree')
        ### add correction by orientaion
        # await self.get_orientation()
        # self.before_rotation_orientation = self.orientation

        try:
            await self.motion.rotate(
                angle = 180.0,
                angular_speed = ROTATING_ANGULAR_SPEED,
                enable_obstacles=False,
                wait=True)
            
        except Exception as error:
            self.log.error(
                f'180 rotation failed, Exception type: '
                f'{type(error)}, Exception: {error}')
            raise error     
        

     

    # async def pushing_cart_identifier(self):
    #     await asyncio.sleep(0.02)
        
    async def _cart_max_distance_verification (self):
        ##TODO add distance from cart by approach input 
        if self.dl > CART_MAX_DISTANCE and self.dr > CART_MAX_DISTANCE:
            self.log.error(f'cart is too far, distance: left: {self.dl} cm, right: {self.dr}')
            self.abort(*ERROR_CART_NOT_ACCESSABLE)


    async def major_angle_correction (self, sign):
        index = 0
        while self.angle > MIN_STARTING_ANGLE or index < MAX_ANGLE_CORRECTION_ATTEMPTS:
            await self.adjust_angle()
            await self.read_srf_values()
            await self.calculate_distance_parameters()
            await self._cart_max_distance_verification()
            index+=1
    


    async def major_angle_identification (self):
        self.log.debug(f'Starting angle: {self.angle}')
        if abs(self.angle) > MIN_STARTING_ANGLE:
            self.log.error(f'Starting angle too high: {self.angle}, adjust angle')
            await self.major_angle_correction(self.sign)



    async def read_srf_values(self):
        ## read srf value with the index, the srf of the cart is 5 and 2
        while(True):
            ##TODO Put timeouts
            await asyncio.sleep(0.01)

            srf_right = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_RIGHT] * 100 
            srf_left = self.sensors.get_sensor_value('srf')[SRF_SENSOR_ID_LEFT] * 100
            if( math.isnan(srf_right) and not math.isnan(srf_left)):
                self.log.error('nan value recived from srf')

            if(not math.isnan(srf_right) and not math.isnan(srf_left)):

                if srf_right > MAX_SRF_VALUE:
                    srf_right = MAX_SRF_VALUE
                if srf_left > MAX_SRF_VALUE:
                    srf_left = MAX_SRF_VALUE    
                self.dr = FILTER_WEIGHT*self.dr + (1-FILTER_WEIGHT)*srf_right
                self.dl = FILTER_WEIGHT*self.dl + (1-FILTER_WEIGHT)*srf_left
                self.average_distance = (self.dl + self.dr)/2
                break

        

    async def set_to_diffualt(self):
        self.sign = 1
        self.state = 'idle'
        self.angle = 0
        self.dl = 0
        self.dr = 0
        self.pushing_index = 0
        self.timeout = self.setup_args['timeout']
        self.gripper_state = {'final_position': 0.0,
                            'final_pressure': 0.0,
                            'attempts': 0,
                            'position_reached': False,
                            'pressure_reached': False,
                            'success': False,
                            'timeout_reached': False,
                            'cart_attached': False,
                            'close_to_actual_position' : False}

    async def setup(self):
        self.skill_apr2tags:RayaSkillHandler = \
                self.register_skill(SkillApproachToTags)

        self.arms = await self.enable_controller('arms')
        self.sensors = await self.enable_controller('sensors')
        self.motion = await self.enable_controller('motion')
        # self.max_angle_step = self.setup_args['max_angle_step']
        


    async def main(self):
        ### approach state

        self.log.info('SkillAttachToCart.main')


        # Rotate 180 degree with the back to the cart
        ## and close the cart adapter
        self.start_time = time.time()
        self.timer = self.start_time

        await self.pre_loop_actions()


        while (True):

            await self._timer_update()
            await self._timeout_verification()

            ## Read the srf values and update them
            await self.read_srf_values()
            await self.calculate_distance_parameters()
            await self._cart_max_distance_verification()
            await self.pushing_cart_identifier()

        #     ## Idetify which state you are
            await self.state_classifier()


            if self.state == 'moving':
                await self.move_backwared()
            
            elif self.state == 'attaching':
                await self.attach()

            elif (self.state == 'rotating'):
                await self.adjust_angle()



            elif self.state == 'attach_verification':
                await self.cart_attachment_verification()

            elif self.state == 'finish':
                cart_attached = self.gripper_state['cart_attached']
                self.log.info(f'time to execute: {self.timer}')
                await self.send_feedback('application finished, cart attachment is: '\
                              f'{cart_attached}')
                
                if self.gripper_state['cart_attached'] is False:
                    self.abort(*ERROR_CART_NOT_ATTACHED)
                break


            
            
            await asyncio.sleep(0.2)



    async def finish(self):
        self.log.info('SkillAttachToCart.finish')
        # await self.skill_apr2cart.execute_finish()
