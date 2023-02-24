import PepperAPI
from PepperAPI import Info
import random

def cv_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	if not Info.Request("State", {"name":"Start"}):
		return
	print("\n========= STATE: Start =========")

	# Wait for signal from Kinematics to start hand detection
	Info.Request("TriggerHandDetection")


	# ========= STATE: AnyQuestions =========
	print("\n========= STATE: AnyQuestions =========")
	# TODO: Detect hands

	# If no hands detected, update state AnyQuestions
	NO_HANDS_DETECTED = False
	if NO_HANDS_DETECTED:
		Info.Send("State", {"AnyQuestions": "NoHandsRaised"})

	# Else if hands detected
	else:
		# Update state
		Info.Send("State", {"AnyQuestions": "HandsRaised"})

		hands_info_list, num_hands = generate_random_hands_info()
		# TODO: First, send total number of hands and send to Kinematics
		Info.Send("NumHands", {"value": num_hands})
		# TODO: Then, send info of each hand ony by one
		for (x,y) in hands_info_list:
			Info.Send("RaisedHandInfo", {
				"bounding_box": (x,y,100,120),
				"frame_res": (1920,1080),
				"confidence_score": -1.0 # send float, if no there'll be an error
			})

	# Continue when no hands detected or QnA loop ended
	# (In which case, Kinematics module will update state AnyQuestions to "NoHandsRaised" once all questions have been answered)


	# ========= STATE: NoiseLevel =========
	print("\n========= STATE: NoiseLevel =========")
	# Wait for update on state change
	state_noise_level = Info.Request("State", {"name": "NoiseLevel"})
	while not state_noise_level:
		state_noise_level = Info.Request("State", {"name": "NoiseLevel"})

	# If high noise level, robot makes a joke and loop restarts
	if state_noise_level == "High":
		return


	# ========= STATE: Attentiveness =========
	print("\n========= STATE: Attentiveness =========")
	# TODO: Start attentiveness detection and calculate total engagement score

	# Update state Attentiveness
	# If not attentive, update state, then control will trigger joke or quiz and loop restarts
	total_engagement_score = .8
	threshold = .5
	if total_engagement_score < threshold:
		Info.Send("State", {"Attentiveness": "NotAttentive"})
		return

	# If attentive, update state 
	else:
		Info.Send("State", {"Attentiveness": "Attentive"})


	# ========= STATE: NoQuestionsLoop =========
	print("\n========= STATE: NoQuestionsLoop =========")
	# nothing to be done
	return	


# TODO: This function just generates dummy info and can be deleted
def generate_random_hands_info():
	lst = []
	num_hands = random.randint(2,5)
	for i in range(num_hands):
		x = random.randint(0, 1920-100)
		y = random.randint(0, 1080-120)
		lst.append((x,y))
	return lst, num_hands


if __name__ == "__main__":
	PepperAPI.init("test")
	while True:
		cv_main()
