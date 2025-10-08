
import sys
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
import time

try:
    print("Initializing ChannelFactory...")
    ChannelFactoryInitialize(0, "eth0")
    print("ChannelFactory initialized. Creating SportClient...")
    dog_client = SportClient()
    print("SportClient created. Setting timeout...")
    dog_client.SetTimeout(10.0)
    print("Timeout set. Initializing SportClient...")
    dog_client.Init()
    print("SportClient initialized. Sending Move command...")

    # The Move command does not return a value. Check its effect by subsequent state.
    dog_client.Move(0.0, 0.0, 0.0)
    print(f"Move command sent: x=0.0, y=0.0, yaw=0.0. Waiting for 1 second...")
    time.sleep(0.5)
    print("Move command finished.")
except Exception as e:
    print(f"Error in move_dog.py: {e}", file=sys.stderr)
    sys.exit(1)
