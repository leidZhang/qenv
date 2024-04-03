from typing import Any
from multiprocessing import Queue

def handle_interprocess(data_queue: Queue, item: Any, block=True, timeout=None) -> None:
    try:
        data_queue.put(item, block, timeout)
    except Queue.Full:
        data_queue.get_nowait()
        data_queue.put(item, block, timeout)
