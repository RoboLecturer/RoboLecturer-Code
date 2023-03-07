import PepperAPI
from PepperAPI import Info

data = {
        "bounding_box": (1,1,1,1),
        "frame_res": (1920,1080),
        "confidence_score": 0
        }
PepperAPI.init("cv")
Info.Send("NumHands", {"value":1})
Info.Send("RaisedHandInfo", data)
