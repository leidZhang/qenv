from typing import Any
from queue import Full, Queue

def handle_full_queue(data_queue: Queue, data: Any):
    try:
        data_queue.put_nowait(data)
    except Full:
        data_queue.get()
        data_queue.put_nowait(data)
