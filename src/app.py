from raya.application_base import RayaApplicationBase

from skills.attach_to_cart import SkillAttachToCart


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.info(f'RayaApplication.setup')
        self.skill_att2cart = self.register_skill(SkillAttachToCart)
        


    async def cb_skill_done(self, exception, result):
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

        await self.skill_att2cart.run(
            setup_args={
                'identifier': [1, 0],
                'tags_size': 0.06,
            }
        )



    async def finish(self):
        self.log.info(f'RayaApplication.finish')
