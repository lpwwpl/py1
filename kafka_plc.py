from PyQt5.QtCore import *
from kafka import KafkaConsumer
import kdata

class Kafaka(QThread):
    signalupdatePLCSig = pyqtSignal()
    def __init__(self, parent=None):
        super(Kafaka, self).__init__(parent)
        self.working = True
        self.consumer = KafkaConsumer('jqr_tp_0608', bootstrap_servers=['113.31.111.188:9092'],consumer_timeout_ms=1000)

    def __del__(self):
        self.working = False

    def run(self):
        # consumer = KafkaConsumer('jqr_tp_0608', bootstrap_servers=['113.31.111.188:9092'])
        for msg in self.consumer:
            if(self.working == False):
                self.consumer.pause()
                break
            if(msg == {}):
                continue
            if msg.key == b'jqr':
                kdata.jqr_msg = msg.value
                # self.signalupdatePLCSig.emit()
            else:
                pass