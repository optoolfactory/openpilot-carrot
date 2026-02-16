import time
import traceback

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service

from openpilot.selfdrive.carrot.carrot_main_sub.carrot_serv import CarrotServ
from openpilot.selfdrive.carrot.carrot_main_sub.udp_rx import UdpReceiver
from openpilot.selfdrive.carrot.carrot_main_sub.kisa_rx import KisaReceiver
from openpilot.selfdrive.carrot.carrot_main_sub.route_engine import RouteEngine
from openpilot.selfdrive.carrot.carrot_main_sub.speed_serv import SpeedService
from openpilot.selfdrive.carrot.carrot_main_sub.broadcast import Broadcaster

from openpilot.selfdrive.carrot.carrot_main_sub.tmux_uploader import TmuxUploader
from openpilot.selfdrive.carrot.carrot_main_sub.panda_debug import PandaDebugRunner
from openpilot.selfdrive.carrot.carrot_main_sub.zmq_cmd import ZmqCmdServer

class CarrotManApp:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.gps_service = get_gps_location_service(self.params)

    self.sm = messaging.SubMaster([
      'deviceState', 'carState', 'controlsState', 'radarState',
      'longitudinalPlan', 'modelV2', 'selfdriveState', 'carControl',
      'navRouteNavd', self.gps_service, 'navInstruction'
    ])
    self.pm = messaging.PubMaster(['carrotMan', 'navRoute', 'navInstructionCarrot'])

    self.carrot_serv = CarrotServ()

    self.route_engine = RouteEngine(self.params, self.params_memory, self.sm, self.pm, self.carrot_serv, self.gps_service)
    self.speed_serv = SpeedService(self.params, self.params_memory, self.sm, self.carrot_serv)

    self.udp_rx = UdpReceiver(self.carrot_serv, port=7706)
    self.kisa_rx = KisaReceiver(self.carrot_serv, port=12345)

    self.tmux = TmuxUploader(self.params, self.params_memory)
    self.panda_debug = PandaDebugRunner()
    self.panda_debug.start()

    self.zmq = ZmqCmdServer(self.params, self.sm, self.tmux, self.panda_debug)

    self.broadcaster = Broadcaster(
      self.params, self.params_memory, self.sm, self.pm,
      self.carrot_serv, self.route_engine, self.speed_serv,
      udp_rx=self.udp_rx, gps_service=self.gps_service,
      broadcast_port=7705
    )

  def start(self):
    self.route_engine.start()
    self.udp_rx.start()
    self.kisa_rx.start()
    self.zmq.start()
    self.broadcaster.start()

def main():
  print("CarrotManager Started")
  app = CarrotManApp()
  app.start()
  while True:
    time.sleep(1)

if __name__ == "__main__":
  try:
    main()
  except Exception:
    traceback.print_exc()
