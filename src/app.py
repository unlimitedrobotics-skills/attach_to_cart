from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart
from skills.approach_to_tags import SkillApproachToTags


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info(f'RayaApplication.setup')
        self.skill_att2cart = self.register_skill(SkillAttachToCart)
        # self.skill_approach = self.register_skill(SkillApproachToTags)
        


    async def cb_skill_done(self, exception, result):
        # self.log.info(f'cb_skill_done!!!!! exception: {type(exception)}')
        self.log.info(f'cb_skill_done, cart attached, result: {result}')
        if exception is None:
            await self.skill_att2cart.execute_finish()
        else: 
            self.log.warn(
                    'error occured while attaching, exception type: '
                    f'{type(exception)} {exception}'
                )


    async def cb_skill_feedback(self, feedback):
        
        self.log.info(feedback)


    async def main(self):
        # self.log.info(f'RayaApplication.loop')
        # await self.skill_approach.run(
        #     setup_args={
        #         'working_camera': 'nav_bottom',
        #         'identifier': [0, 1],
        #         'tags_size': 0.06,
        #         'distance_to_goal': 1.0,
        #     }
        # )
        # await self.skill_approach.run(
        #     setup_args={
        #         'working_camera': 'nav_top',
        #         'identifier': [0, 1],
        #         'tags_size': 0.06,
        #         'distance_to_goal': 0.5,
        #         'intersection_threshold': 1.0
        #     }
        # )
        await self.skill_att2cart.run()



    async def finish(self):
        self.log.info(f'RayaApplication.finish')
