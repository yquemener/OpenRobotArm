import threading
import queue
import time


class ThreadedController:
    def __init__(self, form):
        self.instruction_queue = queue.Queue()
        self.stop_flag = threading.Event()
        self.worker_thread = threading.Thread(target=self._send_instructions_loop)
        self.worker_thread.start()
        self.queue_lock = threading.Lock()
        self.form = form

    def send_pose(self, *args):
        all_args = list(args) + [0]*(8-len(args))
        for i,name in enumerate(['base', 'shoulder', 'elbow', 'wrist_pitch']):
            self.form.jointsSlider[name].slider.setValue(int(all_args[i]))

        if all_args[6]==1:
            self.form.pump_button.setChecked(True)
        else:
            self.form.pump_button.setChecked(False)
    def _send_instructions_loop(self):
        while not self.stop_flag.is_set():
            try:
                instruction = self.instruction_queue.get(timeout=1)
                print(instruction)
                if instruction[0] == "pose":
                    self.send_pose(*instruction[1:])
                elif instruction[0] == "sleep":
                    time.sleep(instruction[1])
                self.instruction_queue.task_done()
            except queue.Empty:
                pass

    def add_instructions(self, instructions):
        with self.queue_lock:
            for instruction in instructions:
                self.add_instruction(instruction)

    def add_instruction(self, instruction):
        self.instruction_queue.put(instruction)

    def stop(self):
        self.stop_flag.set()
        self.worker_thread.join()