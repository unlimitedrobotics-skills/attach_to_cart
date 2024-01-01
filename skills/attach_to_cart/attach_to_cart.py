from raya.skills import RayaSkill, RayaSkillHandler
from raya.controllers import MotionController
from raya.application_base import RayaApplicationBase
import asyncio
from .constants import *




from skills.approach_to_tags import SkillApproachToTags
import math

### TODO fix parameters to defult
### add timeout to each function
### add statement for decrease values in SRF
### add correction after failure

class SkillAttachToCart(RayaSkill):


    DEFAULT_SETUP_ARGS = {
            'distance_before_attach': 0.5,
            'distance_first_approach':1.0,
            'max_angle_step': 15.0
            }
    REQUIRED_SETUP_ARGS = {
        'identifier',
        'tag_size'
    }
    

    async def calculate_distance_parameters(self):
        ## calculate the distance of each dl and dr (distances) and calculate the angle of oreitattion related to the normal to the cart
        if (self.dl > self.dr):
            self.sign = -1
        else:
            self.sign = 1

        self.delta = self.dl - self.dr
        self.angle  = math.tan(self.delta/self.base)/math.pi *180
        
    
    async def state_classifier(self):
        ### change to parameters
        ## rotating state 
        if ((self.dl<ROTATING_DISTANCE or self.dr<ROTATING_DISTANCE) and\
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
        
    ## TODO adjust
    async def adjust_angle(self):
        ## Control law for minimzing the angle between the cart to the robot
        if (self.angle > self.max_angle_step):
            self.angle = self.max_angle_step
        is_moving = self.motion.is_moving()

        if (is_moving):
            await self.motion.cancel_motion()

        await self.motion.rotate(
            angle= abs(self.angle)/360 * 100,
            angular_speed= self.sign * ROTATING_ANGULAR_SPEED,
            enable_obstacles=False,
            wait=True)
            
        self.log.info("finish rotate")

    async def gripper_state_classifier(self):
        ### TODO add position value check
        if (self.gripper_state['pressure_reached'] == True and\
            self.gripper_state['position_reached'] == False):
                
                self.gripper_state['cart_attached'] = True
        else:

            self.gripper_state['cart_attached'] = False
            self.state = 'finish'

    async def cart_attachment_verification(self):
        self.log.info('run cart_attachment_verification')
        verification_dl=self.dl
        verification_dr=self.dr
        verification_delta = verification_dl - verification_dr
        verification_angle  = abs(math.tan(verification_delta/DISTANCE_BETWEEN_SRF_SENSORS)/math.pi *180)

        await self.motion.set_velocity(
                x_velocity = LINEAR_MOVING_VELOCITY,
                y_velocity = 0.0,
                angular_velocity=0.0,
                duration=2.0,
                enable_obstacles=False,
                wait=False, 
                )
        while (self.motion.is_moving()):
            dl=self.sensors.get_sensor_value('srf')['3'] * 100
            dr=self.sensors.get_sensor_value('srf')['0'] * 100
            delta = dl - dr
            angle  = abs(math.tan(delta/DISTANCE_BETWEEN_SRF_SENSORS)/math.pi * 180)
            
            if abs(angle-verification_angle) > VERIFICATION_ANGLE or\
                  verification_dl-dl > VERIFICATION_DISTANCE or\
                      verification_dr-dr > VERIFICATION_DISTANCE:
                self.cart_status['cart_attached'] = False
                break

            await asyncio.sleep(0.2)

        self.log.info('finish cart attach verification')
        self.state = 'finish'
        


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
            # self.log.info(gripper_result)
            await self.gripper_feedback_cb(gripper_result)
            await self.gripper_state_classifier()
            cart_attached = self.gripper_state['cart_attached']
            self.log.info(f'gripper attachment feedback is: {cart_attached}')

            if cart_attached:
                self.state = 'attach_verification'
            else:
                self.state = 'finish'
            

        # except RayaApplicationNotRegistered:
        #     pass
        except Exception as error:
            self.log.error(
                f'gripper attachment failed, Exception type: '
                f'{type(error)}, Exception: {error}')
            raise error

    async def gripper_feedback_cb(self, gripper_result):
        self.gripper_state['final_position'] =  gripper_result['final_position']
        self.gripper_state['final_pressure'] = gripper_result['final_pressure']
        self.gripper_state['position_reached'] = gripper_result['position_reached']
        self.gripper_state['pressure_reached'] = gripper_result['pressure_reached']
        self.gripper_state['success'] = gripper_result['success']
        self.gripper_state['timeout_reached'] = gripper_result['timeout_reached']


    async def move_backwared(self):
        ### TODO try axcept
        await self.motion.set_velocity(
                    x_velocity= -1 * LINEAR_MOVING_VELOCITY,
                    y_velocity=0.0,
                    angular_velocity=0.0,
                    duration=2.0,
                    enable_obstacles=False,
                    wait=False,
                )

        


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
                                                    wait=True,
                                                )
            # self.log.info('gripper_result: ',type(gripper_result))
            # await self.gripper_feedback_cb(gripper_result)
            # print(gripper_result)
        # except RayaApplicationNotRegistered:
        #     pass
        except Exception as error:
            self.log.error(
                f'gripper open to pre-grab position failed, Exception type: '
                f'{type(error)}, Exception: {error}')
            raise error

        ## run approach to tag
        # try:
        #     chest_approach_result = await self.skill_apr2tags.run(
        #             setup_args={
        #                     'working_camera': 'nav_bottom',
        #                     'identifier': [0, 1],
        #                     'tags_size': self.tag_size,
        #                     'distance_to_goal': self.distance_first_approach,
        #                 }
        #         )
        # except Exception as error:
        #     self.log.error(
        #         f'approach to {self.distance_before_attach} meter failed, Exception type: '
        #         f'{type(error)}, Exception: {error}')
        #     self.pre_loop_finish = False
        #     raise error
            
            
        
        # try:    
        #     chin_approach_result = await self.skill_apr2tags.run(
        #         setup_args={
        #                 'working_camera': 'nav_top',
        #                 'identifier': [0, 1],
        #                 'tags_size': self.tag_size,
        #                 'distance_to_goal': self.distance_before_attach,
        #                 'intersection_threshold': 1.0
        #             }
        #     )
        # except Exception as error:
        #     self.log.error(
        #         f'approach to {self.distance_before_attach} meter failed, Exception type: '
        #         f'{type(error)}, Exception: {error}')
        #     self.pre_loop_finish = False
        #     raise error
        
        ##TODO need to remove comment
        # try:
        #     await self.motion.rotate(
        #         angle= 180,
        #         angular_speed= 10,
        #         enable_obstacles=False,
        #         wait=True)
        # except Exception as error:
        #     self.log.error(
        #         f'180 degree rotation failed, Exception type: '
        #         f'{type(error)}, Exception: {error}')
        #     self.pre_loop_finish = False
        #     raise error
        
        return self.pre_loop_finish
            
    async def read_srf_values(self):
        ## read srf value with the index, the srf of the cart is 5 and 2
        self.dl=self.sensors.get_sensor_value('srf')['5'] * 100
        self.dr=self.sensors.get_sensor_value('srf')['2'] * 100
        # self.log.info(f'srf1:{self.dl}, srf2:{self.dr}')


    async def setup(self):
        self.skill_apr2tags:RayaSkillHandler = \
                self.register_skill(SkillApproachToTags)

        self.arms = await self.enable_controller('arms')
        self.sensors = await self.enable_controller('sensors')
        self.motion = await self.enable_controller('motion')

        self.distance_before_attach = self.setup_args['distance_before_attach']
        self.distance_first_approach = self.setup_args['distance_first_approach']
        self.max_angle_step = self.setup_args['max_angle_step']
        self.tag_size = self.setup_args['tag_size']
        self.identifier = self.setup_args['identifier']
        self.sign = 1
        self.state = 'idle'
        self.angle = 0

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


        # Rotate 180 degree with the back to the cart
        ## and close the cart adapter

        await self.pre_loop_actions()

        while (True):
            ## Read the srf values and update them
            await self.read_srf_values()

            await self.calculate_distance_parameters()
        #     ## Idetify which state you are
            await self.state_classifier()

            if self.state == 'moving':
                await self.move_backwared()
                
            elif (self.state == 'rotating'):
                await self.adjust_angle()

            elif self.state == 'attaching':
                await self.attach() 

            elif self.state == 'attach_verification':
                await self.cart_attachment_verification()

            elif self.state == 'finish':
                cart_attached = self.gripper_state['cart_attached']
                self.log.info('application finished, cart attachment is: '\
                              f'{cart_attached}')
                break
                
            await asyncio.sleep(0.2)



    async def finish(self):
        self.log.info('SkillAttachToCart.finish')
        # await self.skill_apr2cart.execute_finish()
