import asyncio
from  project.interface.Task import TaskType      
from project.task_defs.TrackTask import TrackTask
from project.task_defs.DetectTask import DetectTask
from project.task_defs.AvoidTask import AvoidTask
from project.task_defs.TestTask import TestTask
import queue
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

    
class TaskManager():
    
    def __init__(self, drone, compute, transit_map, task_arg_map):
        super().__init__()
        self.trigger_event_queue = queue.Queue()
        self.drone = drone
        self.compute = compute
        self.start_task_id = None
        self.curr_task_id = None
        self.transit_map = transit_map
        self.task_arg_map = task_arg_map

    ######################################################## TASK #############################################################
    def get_current_task(self):
        return self.curr_task_id
            
    def retrieve_next_task(self, current_task_id, triggered_event):
        logger.info(f"TaskManager: next task, current_task_id {current_task_id}, trigger_event {triggered_event}")
        try:
            next_task_id  = self.transit_map.get(current_task_id)(triggered_event)
        except Exception as e:
            logger.info(f"{e}")
                
        logger.info(f"TaskManager: next_task_id {next_task_id}")
        return next_task_id
    
    def transit_task_to(self, task):
        logger.info(f"TaskManager: transit to task with task_id: {task.task_id}, current_task_id: {self.curr_task_id}")
        self.stop_task()
        self.start_task(task)
        self.curr_task_id = task.task_id
    
    async def init_task(self):
        logger.info('TaskManager: Start task')
        self.start_task_id = self.retrieve_next_task("start", None)
        logger.info('TaskManager: Create task')
        start_task = self.create_task(self.start_task_id)
        logger.info('TaskManager: Got task, starting...')
        if start_task != None:
            # set the current task
            self.curr_task_id = start_task.task_id
            logger.info(f"TaskManager: start task, current taskid:{self.curr_task_id}\n")
            
            # takeoff
            await self.drone.takeOff()
            logger.info("TaskManager: taking off")
            
            # start
            self.start_task(start_task)
    
    def create_task(self, task_id):
        logger.info(f'TaskManager: taskid{task_id}')
        if (task_id in self.task_arg_map.keys()):
            if (self.task_arg_map[task_id].task_type == TaskType.Detect):
                logger.info('TaskManager: Detect task')
                return DetectTask(self.drone, self.compute, task_id, self.trigger_event_queue, self.task_arg_map[task_id])
            elif (self.task_arg_map[task_id].task_type == TaskType.Track):
                logger.info('TaskManager: Track task')
                return TrackTask(self.drone, self.compute, task_id, self.trigger_event_queue, self.task_arg_map[task_id])
            elif (self.task_arg_map[task_id].task_type == TaskType.Avoid):
                logger.info('TaskManager: Avoid task')
                return AvoidTask(self.drone, self.compute, task_id, self.trigger_event_queue, self.task_arg_map[task_id])
            elif (self.task_arg_map[task_id].task_type == TaskType.Test):
                logger.info('TaskManager: Test task')
                return TestTask(self.drone, None, task_id, self.trigger_event_queue, self.task_arg_map[task_id])
        return None
    
    def stop_task(self):
        logger.info(f'TaskManager: Stopping current task!')
        if self.taskCurrentCoroutinue is not None:
            # stop all the transitions of the task
            self.currentTask.stop_trans()
            logger.info(f'TaskManager: transitions in the current task stopped!')
            
            is_canceled = self.taskCurrentCoroutinue.cancel()
            if is_canceled:
                logger.info(f'TaskManager:  task cancelled successfully')
                
    def start_task(self, task):
        logger.info(f'TaskManager: start the task! task: {str(task)}')
        self.currentTask = task
        self.taskCurrentCoroutinue = asyncio.create_task(self.currentTask.run())

    def pause_task(self):
        pass

    def force_task(self, task):
        pass

    ######################################################## MAIN LOOP ############################################################
    async def run(self):
        try:
            # start the mc
            logger.info("TaskManager: start the mission runner\n")
            
            logger.info("TaskManager: start the task\n")
            await self.init_task()
            
            # main
            logger.info("TaskManager: go to the inf loop routine\n")
            while True:
                if (not self.trigger_event_queue.empty()):
                    item = self.trigger_event_queue.get()
                    task_id = item[0]
                    trigger_event = item[1]
                    logger.info(f"TaskManager: Trigger one event! \n")
                    logger.info(f"TaskManager: Task id  {task_id} \n")
                    logger.info(f"TaskManager: event   {trigger_event} \n")
                    if (task_id == self.get_current_task()):
                        next_task_id = self.retrieve_next_task(task_id, trigger_event)
                        if (next_task_id == "terminate"):
                            break
                        else:
                            next_task = self.create_task(next_task_id)
                            logger.info(f"TaskManager: task created  taskid {str(next_task.task_id)} \n")
                            self.transit_task_to(next_task, self.tr)
                            
                await asyncio.sleep(0.1)            

        except asyncio.CancelledError as e:
            logger.info(f"TaskManager: catching the asyncio exception {e} \n")
        finally:
            # stop the current task
            self.stop_task()
            #end the tr
            logger.info("TaskManager: terminate the runner\n")

    
