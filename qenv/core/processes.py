from typing import Any
from queue import Full
from multiprocessing.queues import Queue

def handle_interprocess(data_queue: Queue, item: Any, block=True, timeout=None) -> None:
    if data_queue.qsize() == data_queue._maxsize:
        data_queue.get_nowait()
    data_queue.put(item, block, timeout)
