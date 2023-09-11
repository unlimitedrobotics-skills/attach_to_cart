import math
import queue
import time
import asyncio
import numpy as np
from functools import partial
import tf_transformations as tftr
from transforms3d import euler

from raya.controllers import MotionController
from raya.controllers import CVController
from raya.skills import RayaFSMSkill
from raya.exceptions import RayaMotionObstacleDetected
from .constants import *



class SkillApproachToTags(RayaFSMSkill):

    ### SKILL ###

    REQUIRED_SETUP_ARGS = [
            'working_cameras',
            'tags_size',
            'identifier'
        ]

    DEFAULT_SETUP_ARGS = {
            'fsm_log_transitions':True,
        }

    REQUIRED_EXECUTE_ARGS = []

    DEFAULT_EXECUTE_ARGS = {
            'distance_to_goal': 0.5,
            'angle_to_goal': 0.0,
            'angular_velocity': 10,
            'linear_velocity': 0.1,
            'min_correction_distance': 0.5,
            'step_size': 0.2,
            'tags_to_average': 6,
            'max_x_error_allowed': 0.02,
            'max_y_error_allowed': 0.05,
            'max_angle_error_allowed': 5.0,
            'allowed_motion_tries': 10,
            'max_allowed_distance': 2.5,
        }

    ### FSM ###

    STATES = [
            'READ_APRILTAG',
            'GO_TO_INTERSECTION',
            'READ_APRILTAG_2',
            'ROTATE_TO_APRILTAGS',
            'ROTATE_UNTIL_DETECTIONS',
            'STEP_N',
            'READ_APRILTAGS_N',
            'READ_APRILTAGS_N_2',
            'ROTATE_TO_APRILTAGS_N',
            'ROTATE_UNTIL_DETECTIONS_N',
            'CENTER_TO_TARGET',
            'READ_APRILTAGS_FINAL_CORRECTION',
            'MOVE_LINEAR_FINAL',
            'READ_APRILTAGS_FINAL',
            'END'
        ]

    INITIAL_STATE = 'READ_APRILTAG'

    END_STATES = [
            'END',
        ]

    STATES_TIMEOUTS = {
            'READ_APRILTAG' :      
                    (NO_TARGET_TIMEOUT_LONG, ERROR_NO_TARGET_FOUND),
            'READ_APRILTAGS_N' :   
                    (NO_TARGET_TIMEOUT_LONG, ERROR_NO_TARGET_FOUND),
            'READ_APRILTAGS_FINAL_CORRECTION': 
                    (NO_TARGET_TIMEOUT_LONG, ERROR_NO_TARGET_FOUND),
            'READ_APRILTAGS_FINAL': 
                    (NO_TARGET_TIMEOUT_LONG, ERROR_NO_TARGET_FOUND),
        }

    ### SKILL METHODS ###

    async def setup(self):
        self.timer1 = None
        self.step_task = None

        self.motion:MotionController = await self.get_controller('motion')
        self.cv:CVController = await self.get_controller('cv')
        model_params = {
                'families' : 'tag36h11',
                'nthreads' : 4,
                'quad_decimate' : 2.0,
                'quad_sigma': 0.0,
                'decode_sharpening' : 0.25,
                'refine_edges' : 1,
                'tag_size' : self.setup_args['tags_size'],
            }
        self.predictors={}
        for camera in self.setup_args['working_cameras']:
            self.predictors[camera] = await self.cv.enable_model(
                    name='apriltags', 
                    source=camera,
                    model_params = model_params
                )
        self.waiting_detection = False
        for camera in self.predictors:
            self.predictors[camera].set_detections_callback(
                    callback=partial(
                            self._callback_predictions,
                            camera,
                        ),
                    as_dict=True,
                    call_without_detections=True
            )
        

    async def finish(self):
        for camera in self.setup_args['working_cameras']:
            await self.cv.disable_model(model_obj=self.predictors[camera])


    ### HELPERS ###


    def setup_variables(self):
        self.handler_names = HANDLER_NAMES
        for camera in self.predictors:
            self.handler_name = type(self.predictors[camera]).__name__
            break
        self.approach = self.handler_names[self.handler_name]
        
        #flags
        self.is_there_detection = False
        self.waiting_detection = True
        self.wait_until_complete_queue = True
        self.is_final_step = False

        #calculations
        self.detections_cameras = set()
        self.correct_detection = None
        self.angle_intersection_goal = None
        self.angle_robot_intersection = None
        self.angular_sign = None
        self.angle_robot_goal = None
        self.linear_distance = None
        self.tries = 0
        
        self.__predictions_queue = queue.Queue()

        self.additional_distance = \
                self.execute_args['min_correction_distance']

    
    def validate_arguments(self):
        self.setup_variables()
        if self.execute_args['angle_to_goal'] > 180.0 \
            or self.execute_args['angle_to_goal'] < -180.0:
            self.abort(*ERROR_INVALID_ANGLE)
        if not self.handler_name in HANDLER_NAMES:
            self.abort(*ERROR_INVALID_PREDICTOR)
        if self.setup_args['identifier'] is None and \
                HANDLER_NAMES[self.handler_name] is not None:
            self.abort(*ERROR_IDENTIFIER_NOT_DEFINED)
        if len(self.setup_args['identifier'])>2:
            self.abort(*ERROR_IDENTIFIER_LENGTH_HAS_BEEN_EXCEED)


    async def rotate_and_move_linear(self):
        await self.send_feedback({
                'rotation': self.angular_sign* self.angle_robot_intersection,
                'linear': self.linear_distance
            })
        if abs(self.projected_error_y) > self.execute_args['max_y_error_allowed']:
            await self.motion.rotate(
                    angle=abs(self.angle_robot_intersection), 
                    angular_speed=self.execute_args['angular_velocity'] * self.angular_sign, 
                    wait=True,
                )
        await self.motion.move_linear(
                distance=self.linear_distance, 
                x_velocity=self.execute_args['linear_velocity'], 
                wait=True,
            )
    

    def start_detections(self, wait_complete_queue=True):
        self.is_there_detection = False
        self.waiting_detection = True
        self.wait_until_complete_queue = wait_complete_queue
        self.correct_detection = None
        self.detections_cameras = set()


    def stop_detections(self):
        self.waiting_detection = False

    
    async  def check_initial_position(self):
        x_final = False
        y_final = False
        robot_position = [0, 0, 0]
        self.initial_pos = self.correct_detection
        distance_x, distance_y = self.get_relative_coords(
            self.correct_detection[:2], robot_position[:2], 
            self.correct_detection[2])
        ini_target_distance = self.get_euclidean_distance(
            robot_position[:2], self.correct_detection)
        await self.send_feedback({
                'detected_cameras': self.detections_cameras,
                'initial_euclidean_distance': ini_target_distance,
                'initial_error_x' : distance_x,
                'initial_error_y' : distance_y
            })
        distance_x = abs(distance_x - self.execute_args['distance_to_goal'])
        
        if abs(distance_x) <= X_THRESHOLD_ERROR:
            x_final = True                         
        if abs(distance_y) <= Y_THRESHOLD_ERROR:
            y_final = True
        if x_final == True and y_final == True:  
            return True 
        
        if ini_target_distance < (self.execute_args['distance_to_goal'] + 
                                  self.execute_args['min_correction_distance']):
            self.abort(
                    ERROR_TOO_CLOSE_TO_TARGET,
                    f'Robot is too close to the target. It is '
                    f'{ini_target_distance:.2f}, and it must be at least the '
                    f'distance to goal ({self.execute_args["distance_to_goal"]:.2f}) '
                    f'+ MIN_CORRECTION_DISTANCE ({self.execute_args["min_correction_distance"]})'
                )
            
        if ini_target_distance > self.execute_args['max_allowed_distance']:
            self.abort(
                    ERROR_TOO_FAR_TO_TARGET,
                    f'Robot is too far to the target. It is '
                    f'{ini_target_distance:.2f}, and it must be less than '
                    f'max_allowed_distance '
                    f'{self.execute_args["max_allowed_distance"]}'
                )
            

    async def planning_calculations(self):
        (self.angular_sign, self.distance, self.distance_to_inter, 
         self.angle_robot_intersection,
         self.angle_intersection_goal) = \
             self.get_intersection_info()
        self.angle_robot_goal = self.get_angle(
                [0,0,0], 
                self.correct_detection
            )
        self.projected_error_y = await self.get_error_projection_y()
        

    async def get_error_projection_y(self):
        original_translation = (self.correct_detection[0], 
                                self.correct_detection[1], 
                                0.0)
        original_rotation = (0, 0, math.radians(self.correct_detection[2]))  # 45 degrees in radians
        original_matrix = tftr.compose_matrix(translate=original_translation, angles=original_rotation)
        inverse_matrix = tftr.inverse_matrix(original_matrix)
        inverse_translation = tftr.translation_from_matrix(inverse_matrix)
        inverse_rotation = tftr.euler_from_matrix(inverse_matrix)
        
        error_y = inverse_translation[1] + np.sign(inverse_rotation[2]) * \
        np.tan(np.pi-abs(inverse_rotation[2]))*(inverse_translation[0] -
        self.execute_args['distance_to_goal'])
        await self.send_feedback({'proyected_y_error': error_y})
        return error_y

    def get_intersection_info(self):
        line_2 = self.__get_proyected_point(
            self.correct_detection[0], self.correct_detection[1], 
            self.correct_detection[2], 
            self.execute_args['distance_to_goal']+self.additional_distance)
        robot_point= [0,0,0]
        p1 = np.array(self.correct_detection[:2])
        p2 = np.array(line_2)
        p0 = np.array(robot_point[:2])
        before = False
    
        left_side = -1
        line_direction = p2 - p1
        intersection = p2
        if intersection[1] > 0:
            left_side = 1
        intersection_direction = np.dot(line_direction, intersection - p0)
        if intersection_direction >= 0:
            before = True
        distance_intersection = self.get_euclidean_distance(intersection, p0)
        min_distance_intersection = self.get_distance_intersection(
                    self.correct_detection[:2], intersection)
        angle_robot_intersection = np.arccos(
            np.dot(np.array([1,0]),(p2-p0)/np.linalg.norm(p2-p0)))

        angle_intersection_goal = -np.sign(min_distance_intersection)*\
            (np.pi-abs(np.arccos(np.dot((p1-p2)/np.linalg.norm(p1-p2),
                                        (p0-p2)/np.linalg.norm(p0-p2)))))
        
        
        return ( left_side, 
                distance_intersection, abs(min_distance_intersection), 
                math.degrees(angle_robot_intersection),
                math.degrees(angle_intersection_goal))
    
    
    def _callback_predictions(self, camera, predictions, timestamp):
        try:
            if predictions and self.waiting_detection:
                predictions['camera'] = camera
                self.__predictions_queue.put(predictions)
                if self.__predictions_queue._qsize() == \
                        self.execute_args['tags_to_average'] or \
                        not self.wait_until_complete_queue:                
                    self.__update_predictions()
        except Exception as e:
            self.abort(254, f'Exception in callback: [{type(e)}]: {e}')
            import traceback
            traceback.print_exc()
                

    def __update_predictions(self ):
        predicts = []
        temporal_queue = queue.Queue()
        
        while not self.__predictions_queue.empty():
            prediction = self.__predictions_queue.get()
            goal = self.__proccess_prediction(prediction)
            if goal is None:
                continue
            temporal_queue.put(prediction)
            predicts.append(goal)
        self.robot_position=()

        if (len(predicts) == self.execute_args['tags_to_average'] or 
                not self.wait_until_complete_queue):
            correct_detection=self.__process_multiple_detections(predicts)
            if correct_detection:
                self.correct_detection = correct_detection
                self.is_there_detection = True
                self.waiting_detection = False
                while not temporal_queue.empty():
                    self.detections_cameras.add(temporal_queue.get()['camera'])
                return
            else:
                if not temporal_queue.empty():
                    temporal_queue.get() # discarding last value 

        while not temporal_queue.empty():
            self.__predictions_queue.put(temporal_queue.get())


    def __proccess_prediction(self, prediction):
        predicts=[]
        list_size = len(self.setup_args['identifier']) 
        ids = [int(id) for id in self.setup_args['identifier']]
        for pred in prediction.values():
            if type(pred) == str:
                continue
            if int(pred[self.approach]) not in ids:
                continue
            predicts.append(pred)
        if len(predicts) < list_size:
            return None
        predicts_final=[]
        for pred in predicts:
            if pred['pose_base_link']:
                angle=self.__quaternion_to_euler(pred['pose_base_link'])[2]
                goal = [pred['pose_base_link'].pose.position.x,
                                pred['pose_base_link'].pose.position.y,
                                angle+
                                self.execute_args['angle_to_goal']]
                if list_size == 1:
                    return goal
                predicts_final.append((goal, goal[2]))
            
        goal = self.__process_multiple_tags(predicts_final)       
        return goal 


    def __process_multiple_detections(self, predictions):
        # Step 1: Calculate the mean of the list of predictions (arrays of three values)
        predictions_np = np.array(predictions)
        
        try:
            valid_predictions = predictions_np[~np.isnan(predictions_np).any(axis=1)]
        except np.AxisError:
            return None

        if len(valid_predictions) == 0:
            return None  # Return None if all positions have NaN values

        # Step 2: Calculate the mean of the valid predictions (arrays of three values)
        mean_prediction = np.mean(valid_predictions, axis=0)

        # Step 3: Get the values below the mean
        below_mean_values = valid_predictions[
            (abs(valid_predictions-mean_prediction)<=0.3).sum(axis=1)>1, :]
        below_mean_values = below_mean_values[
            (abs(below_mean_values[:,-1]-mean_prediction[-1])<=10), :]
        if len(below_mean_values) == 0:
            return None
        # Step 5: Calculate the mean of the values below the mean
        mean_below_mean = np.mean(below_mean_values, axis=0)

        return mean_below_mean.tolist()
    

    def __process_multiple_tags(self, predicts):
        pt1 = predicts[0][0]
        pt2 = predicts[1][0]
        angle4 = math.degrees(np.arctan((pt1[1] - pt2[1])/(pt1[0] - pt2[0])))
        angle5 = np.sign(angle4)*(90-abs(angle4))
        final_point = ((pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2, 
                       np.sign(angle5)*(180-abs(angle5))+
                                self.execute_args['angle_to_goal'])
        return final_point  
    

    def __get_proyected_point(self, x, y, angle, distance):
        angulo_rad = math.radians(angle)
        x_final = x + distance * math.cos(angulo_rad)
        y_final = y + distance * math.sin(angulo_rad)
        return (x_final, y_final)


    def get_relative_coords(self, punto_a, punto_b, angulo_direccion):
        x_a, y_a = punto_a
        x_b, y_b = punto_b
        delta_x = x_b - x_a
        delta_y = y_b - y_a
        angulo_rad = math.radians(angulo_direccion)
        x_rel = delta_x * math.cos(angulo_rad) + delta_y * math.sin(angulo_rad)
        y_rel = delta_y * math.cos(angulo_rad) - delta_x * math.sin(angulo_rad)
        return x_rel, y_rel


    def get_euclidean_distance(self, pt1, pt2):
        distance = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
        return distance
    

    def __quaternion_to_euler(self, point_pose):
        x = point_pose.pose.orientation.x
        y = point_pose.pose.orientation.y
        z = point_pose.pose.orientation.z
        w = point_pose.pose.orientation.w
        quat = [w, x, y, z]
        euler_angles = euler.quat2euler(quat, 'sxyz')
        roll = math.degrees(euler_angles[0])
        pitch = math.degrees(euler_angles[1])
        yaw = math.degrees(euler_angles[2])

        return roll, pitch, yaw
    
    def get_distance_intersection(self,punto1, punto2):
        m = (punto2[1] - punto1[1]) / (punto2[0] - punto1[0])
        b = punto1[1] - m * punto1[0]
        return b

    def get_angle(self, robot_point, line_point1):
        robot_angle_rad = np.radians(robot_point[2])
        robot_direction = np.array([np.cos(robot_angle_rad), 
                                    np.sin(robot_angle_rad)])
        line_direction = np.array(line_point1[:2]) - np.array(robot_point[:2])
        line_direction /= np.linalg.norm(line_direction)  # Normalizar el vector de dirección de la línea
        angle_rad = np.arctan2(line_direction[1], 
                               line_direction[0]) - np.arctan2(
            robot_direction[1], robot_direction[0])
        angle_deg = np.degrees(angle_rad)

        return angle_deg
    
    def motion_running(self):
        return self.motion.is_moving()


    ### ACTIONS ###


    async def enter_READ_APRILTAG(self):
        self.validate_arguments()
        self.start_detections()


    async def enter_GO_TO_INTERSECTION(self):
        await self.planning_calculations()
        self.linear_distance= self.distance
        if abs(self.distance_to_inter) > MAX_MISALIGNMENT:
            self.abort(
                    ERROR_TOO_DISALIGNED,
                    'The robot is disaligned by '
                    f'{abs(self.distance_to_inter)} meters, max '
                    f'{MAX_MISALIGNMENT} is allowed.'
                )
        self.step_task = asyncio.create_task(self.rotate_and_move_linear())


    async def enter_READ_APRILTAG_2(self):
        self.start_detections(wait_complete_queue=False)
        self.timer1 = time.time()
        

    async def enter_ROTATE_TO_APRILTAGS(self):
        await self.send_feedback({'rotation':self.angle_intersection_goal})
        await self.motion.rotate(
                angle=self.angle_intersection_goal, 
                angular_speed=self.execute_args['angular_velocity'], 
                wait=False
            )
    

    async def enter_ROTATE_UNTIL_DETECTIONS(self):
        self.start_detections(wait_complete_queue=False)
        ang_vel=(self.execute_args['angular_velocity'] *
                 np.sign(self.angle_intersection_goal))
        await self.motion.set_velocity(x_velocity=0.0,
                                       y_velocity=0.0,
                                       angular_velocity=ang_vel,
                                       duration=0.2,
                                       wait=False)
    

    async def  enter_READ_APRILTAGS_N(self):
        self.start_detections()
    

    async def enter_READ_APRILTAGS_N_2(self):
        self.start_detections(wait_complete_queue=False)
        self.timer1 = time.time()
        

    async def enter_ROTATE_TO_APRILTAGS_N(self):
        await self.send_feedback({'rotation':self.angle_intersection_goal})
        await self.motion.rotate(
                angle=self.angle_intersection_goal, 
                angular_speed=self.execute_args['angular_velocity'], 
                wait=False
            )
        

    async def enter_ROTATE_UNTIL_DETECTIONS_N(self):
        self.start_detections(wait_complete_queue=False)
        ang_vel=(self.execute_args['angular_velocity'] *
                 np.sign(self.angle_intersection_goal))
        await self.motion.set_velocity(x_velocity=0.0,
                                       y_velocity=0.0,
                                       angular_velocity=ang_vel,
                                       duration=0.2,
                                       wait=False)
        

    async def enter_STEP_N(self):
        self.additional_distance = 0.0
        await self.planning_calculations()
        self.linear_distance = self.execute_args['step_size']
        if self.distance<=self.execute_args['step_size']:
            self.is_final_step=True
            self.linear_distance=self.distance
        await self.send_feedback({
                'detected_cameras': self.detections_cameras,
                'distance_to_target': self.distance,
            })
        self.step_task = asyncio.create_task(self.rotate_and_move_linear())


    async def enter_CENTER_TO_TARGET(self):
        await self.planning_calculations()
        await self.send_feedback({
                'detected_cameras': self.detections_cameras,
                'final_rotation': self.angle_robot_goal,
            })
        if abs(self.angle_robot_goal) > \
                self.execute_args['max_angle_error_allowed']:
            await self.motion.rotate(
                    angle=self.angle_robot_goal, 
                    angular_speed=self.execute_args['angular_velocity'], 
                    wait=False
                )
    

    async def enter_READ_APRILTAGS_FINAL_CORRECTION(self):
        self.start_detections()
        self.timer1 = time.time()
        
    
    async def enter_MOVE_LINEAR_FINAL(self):
        await self.planning_calculations()
        distance_x, distance_y = self.get_relative_coords(
            self.correct_detection[:2], [0,0], 
            self.correct_detection[2])
        linear_distance = distance_x - self.execute_args['distance_to_goal']
        await self.send_feedback({
                'detected_cameras': self.detections_cameras,
                "final_linear": linear_distance,
            })
        if abs(linear_distance) > self.execute_args['max_x_error_allowed']:
            await self.motion.move_linear(
                    distance = linear_distance, 
                    x_velocity = self.execute_args['linear_velocity'], 
                    wait=False,
                )
            

    async def enter_READ_APRILTAGS_FINAL(self):
        self.start_detections()
        self.timer1 = time.time()


    ### TRANSITIONS ###
    async def transition_from_READ_APRILTAG(self):
        if self.is_there_detection:
            if await self.check_initial_position():
                self.set_state('CENTER_TO_TARGET')
            else:
                self.set_state('GO_TO_INTERSECTION')


    async def transition_from_GO_TO_INTERSECTION(self):
        if self.step_task.done():
            is_motion_ok = False
            try:
                await self.step_task
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                self.set_state('ROTATE_UNTIL_DETECTIONS')

            if is_motion_ok:
                self.set_state('READ_APRILTAG_2')


    async def transition_from_READ_APRILTAG_2(self):
        if (time.time()-self.timer1) > NO_TARGET_TIMEOUT_SHORT or \
                self.is_there_detection:
            self.stop_detections()
            if self.is_there_detection:
                self.set_state('READ_APRILTAGS_N')
            else:
                self.set_state('ROTATE_TO_APRILTAGS')


    async def transition_from_ROTATE_TO_APRILTAGS(self):
        if not self.motion_running():
            is_motion_ok = False
            try:
                self.motion.check_last_motion_exception()
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                self.set_state('ROTATE_UNTIL_DETECTIONS')
            if is_motion_ok:
                self.set_state('READ_APRILTAGS_N')
    

    async def transition_from_ROTATE_UNTIL_DETECTIONS(self):
        if not self.motion_running():
            is_motion_ok = False
            try:
                self.motion.check_last_motion_exception()
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                
            if is_motion_ok and self.is_there_detection:
                self.set_state('READ_APRILTAG')
            else:
                await self.enter_ROTATE_UNTIL_DETECTIONS()
                
            
    async def transition_from_STEP_N(self):
        if self.step_task.done():
            try:
                await self.step_task
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                self.is_final_step = False

            self.set_state('READ_APRILTAGS_N_2')


    async def transition_from_READ_APRILTAGS_N(self):
        if self.is_there_detection:
            if self.is_final_step:
                self.set_state('CENTER_TO_TARGET')
            else:
                self.set_state('STEP_N')
    

    async def transition_from_READ_APRILTAGS_N_2(self):
        if (time.time()-self.timer1) > NO_TARGET_TIMEOUT_SHORT or \
                self.is_there_detection:
            self.stop_detections()
            if self.is_there_detection:
                self.set_state('READ_APRILTAGS_N')
            else:
                self.set_state('ROTATE_TO_APRILTAGS_N')


    async def transition_from_ROTATE_TO_APRILTAGS_N(self):
        print("transition ROTATE_TO_APRILTAGS_N")
        if not self.motion_running():
            is_motion_ok = False
            try:
                self.motion.check_last_motion_exception()
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                self.set_state('ROTATE_UNTIL_DETECTIONS_N')
            if is_motion_ok:
                self.set_state('READ_APRILTAGS_N')
                

    async def transition_from_ROTATE_UNTIL_DETECTIONS_N(self):
        if not self.motion_running():
            is_motion_ok = False
            try:
                self.motion.check_last_motion_exception()
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                
            if is_motion_ok and self.is_there_detection:
                self.set_state('READ_APRILTAGS_N')
            else:
                await self.enter_ROTATE_UNTIL_DETECTIONS_N()


    async def transition_from_CENTER_TO_TARGET(self):
        if not self.motion_running():
            is_motion_ok = False
            try:
                self.motion.check_last_motion_exception()
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                self.set_state('READ_APRILTAGS_N')
            if is_motion_ok:
                self.set_state('READ_APRILTAGS_FINAL_CORRECTION')

    
    async def transition_from_READ_APRILTAGS_FINAL_CORRECTION(self):
        if self.is_there_detection:
            self.set_state('MOVE_LINEAR_FINAL')


    async def transition_from_MOVE_LINEAR_FINAL(self):
        if not self.motion_running():
            is_motion_ok= False
            try:
                self.motion.check_last_motion_exception()
                is_motion_ok = True
            except RayaMotionObstacleDetected as e:
                self.tries+=1 
                if self.tries >= self.execute_args['allowed_motion_tries']:
                    raise e
                self.set_state('READ_APRILTAGS_FINAL_CORRECTION')
            if is_motion_ok:
                self.set_state('READ_APRILTAGS_FINAL')


    async def transition_from_READ_APRILTAGS_FINAL(self):
        if self.is_there_detection:
            await self.planning_calculations()
            distance_x, distance_y = self.get_relative_coords(
                    self.correct_detection[:2], [0,0], 
                    self.correct_detection[2]
                )
            error_x = distance_x - self.execute_args['distance_to_goal']
            error_angle = self.correct_detection[2] - 180.0
            if error_angle < -180.0: error_angle += 360.0
            elif error_angle > 180.0: error_angle -= 360.0
            self.main_result = {
                    'detected_cameras': self.detections_cameras,
                    "final_error_x": error_x,
                    "final_error_y": distance_y,
                    "final_error_angle": error_angle,
                }
            self.set_state('END')
