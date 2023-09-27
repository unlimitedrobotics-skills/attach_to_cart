from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart
from raya.exceptions import *


class RayaApplication(RayaApplicationBase):

    REQUIRED_SETUP_ARGS = {
        'identifier',
        'tags_size'
    }

    async def setup(self):
        self.log.info(f'RayaApplication.setup')
        self.skill_aborted = False
        self.skill_att2cart = self.register_skill(SkillAttachToCart)
        try:
            await self.skill_att2cart.execute_setup(
                    setup_args={
                            'tags_size': self.tags_size,
                        },
                )
        except RayaSkillAborted as error:
            print (f'error: {error},type: {type(error)}')
            # error_code = error[0]
            # error_msg = error[1]
            # self.log.error(f'error code: {error_code}, error: {error_msg}')


    # async def cb_skill_done(self, exception, result):
    #     self.log.info(f'cb_skill_done, cart attached, result: {result}')
    #     if exception is None:
    #         await self.skill_att2cart.execute_finish()
    #     else: 
    #         self.log.warn(
    #                 'error occured while attaching, exception type: '
    #                 f'{type(exception)} {exception}'
    #             )

    async def main(self):
        try:
            await self.skill_att2cart.execute_main(
                execute_args={
                    'identifier': self.identifier,
                    'pre_att_angle':self.pre_att_angle,
                }
            )
        except RayaSkillAborted as error:
            self.skill_aborted = True
            self.log.error(f'error code: {error.error_code}, error: {error.error_msg}')
            self.log.warn('cart NOT connected')


    async def finish(self):
        if not self.skill_aborted:
            await self.skill_att2cart.execute_finish()

        self.log.info(f'RayaApplication.finish')


    def get_arguments(self):
        self.tags_size = self.get_argument('-s', '--tag-size',
                type=float,
                help='size of tags to be detected',
                required=True
            )
        self.pre_att_angle = self.get_argument('-ra', '--rotation-angle',
                type=float,
                help='pre attach rotation angle',
                required=False
            )
        self.identifier = self.get_argument('-i', '--identifier', 
                type= int,
                list= True, 
                required=True,
                default='',
                help='ids of the apriltags to be used'
            )

    ### callbacks

    async def cb_skill_feedback(self, feedback):
        self.log.info(feedback)
