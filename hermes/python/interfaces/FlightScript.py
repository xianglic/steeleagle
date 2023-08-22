import threading
import ctypes
import queue

class FlightScript(threading.Thread):

    def __init__(self, drone, cloudlet):
        threading.Thread.__init__(self)
        self.drone = drone
        self.cloudlet = cloudlet
        self.taskThread = None
        self.taskQueue = queue.Queue()

    def _execLoop(self):
        try:
            while not self.taskQueue.empty():
                self._exec(self.taskQueue.get())
            self.drone.hover()
        except Exception as e:
            print(f'Exec loop interrupted by exception: {e}')
            self.drone.hover()

    def _exec(self, task):
        self.currentTask = task
        self.taskThread = task
        self.taskThread.start()
        self.taskThread.join()

    def _get_id(self):
        if not self.is_alive():
            raise threading.ThreadError("the thread is not active")

        # do we have it cached?
        if hasattr(self, "_thread_id"):
            return self._thread_id

        # no, look for it in the _active dict
        for tid, tobj in threading._active.items():
            if tobj is self:
                self._thread_id = tid
                return tid

    def _kill(self):
        try:
            self.taskQueue = queue.Queue() # Clear the queue
            self.taskThread.stop()
        except RuntimeError as e:
            print(e)
            self.drone.hover()

    def _pause(self):
        pass

    def _push_task(self, task):
        self.taskQueue.put(task)

    def _force_task(self, task):
        pass
