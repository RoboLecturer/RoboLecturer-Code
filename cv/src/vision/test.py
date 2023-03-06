import PepperAPI
from PepperAPI import Info

PepperAPI.init("cv")
Info.Send("SimpleMsg", {"value": "Hello world"})
