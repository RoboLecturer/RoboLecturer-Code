import PepperAPI
from PepperAPI import Info
import random

def cv_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	# Wait for signal from Kinematics to start hand detection
	Info.Request("TriggerHandDetection")


	# ========= STATE: AnyQuestions =========
	# TODO: Start hand detection
	hands_info_list, num_hands = generate_random_hands_info()

	# If no hands detected, update state and continue
	if num_hands == 0:
		Info.Send("State", {"AnyQuestions": "NoHandsRaised"})

	# Else if hands detected
	else:
		# Update state
		Info.Send("State", {"AnyQuestions": "HandsRaised"})

		# First, send total number of hands to Kinematics
		Info.Send("NumHands", {"value": num_hands})

		# Then, send info of each hand ony by one
		for (x,y) in hands_info_list:
			Info.Send("RaisedHandInfo", {
				"bounding_box": (x,y,100,120),
				"frame_res": (1280,720),
				"confidence_score": -1.0 # send float, if no there'll be an error
			})

		# Wait for end of QnA loop. State updated by Kinematics
		while Info.Request("State", {"name":"AnyQuestions", "print":False}) != "NoHandsRaised":
			pass


	# ========= STATE: NoiseLevel =========
	# Wait for update on state change
	state = Info.Request("State", {"name": "NoiseLevel"})

	# If high noise level, robot makes a joke and loop restarts
	if state == "High":
		return


	# ========= STATE: Attentiveness =========
	# TODO: Start attentiveness detection and classify into attentive or inattentive
	CLASS_IS_ATTENTIVE = random.choice([True, False])
	CLASS_IS_ATTENTIVE = False

	# If not attentive, update state. Loop restarts after control triggers joke/quiz
	if not CLASS_IS_ATTENTIVE:
		Info.Send("State", {"Attentiveness": "NotAttentive"})
		return

	# If attentive, update state 
	else:
		Info.Send("State", {"Attentiveness": "Attentive"})


	# ========= STATE: NoQuestionsLoop =========
	# Nothing to be done. Just wait for state change
	Info.Request("State", {"name": "NoQuestionsLoop"})

	return	


# TODO: This function just generates dummy info and can be deleted
def generate_random_hands_info():
	lst = []
	num_hands = random.randint(0,3)
	num_hands = 2
	lst = [[540,50],[740,50]] # left, right
	# for i in range(num_hands):
	# 	x = random.randint(0, 1920-100)
	# 	y = random.randint(0, 1080-120)
	# 	lst.append((x,y))
	return lst, num_hands


if __name__ == "__main__":
	PepperAPI.init("master")
	while True:
		cv_main()
