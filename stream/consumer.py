import cv2
import numpy as np
import sys
import time

from kombu import Connection, Exchange, Queue
from kombu.mixins import ConsumerMixin

# Default RabbitMQ server URI
rabbit_url = 'amqp://user4:rmq2021@usw-gp-vm.westus.cloudapp.azure.com:5672//'

# Kombu Message Consuming Worker
class Worker(ConsumerMixin):
    def __init__(self, connection, queues):
        self.connection = connection
        self.queues = queues

    def get_consumers(self, Consumer, channel):
        return [Consumer(queues=self.queues,
                         callbacks=[self.on_message],
                         accept=['image/jpeg'])]

    def on_message(self, body, message):
        # get the original jpeg byte array size
        size = sys.getsizeof(body) - 33
        # jpeg-encoded byte array into numpy array
        np_array = np.frombuffer(body, dtype=np.uint8)
        np_array = np_array.reshape((size, 1))
        # # decode jpeg-encoded numpy array 
        image = cv2.imdecode(np_array, 1)

        # # show image
        cv2.imshow("image", image)
        cv2.waitKey(1)
        print(size)

        # send message ack
        message.ack()

def run():
    exchange = Exchange("video-exchange", type="direct")
    queues = [Queue("video-queue", exchange, routing_key="video")]
    with Connection(rabbit_url, heartbeat=4) as conn:
            worker = Worker(conn, queues)
            worker.run()

if __name__ == "__main__":
    run()