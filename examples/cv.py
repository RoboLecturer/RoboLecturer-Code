import PepperAPI
from PepperAPI import Info
import random

def cv_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	if not Info.Request("State", {"name":"Start"}):
		return

	# Wait for signal from Kinematics to start hand detection
	Info.Request("TriggerHandDetection")


	# ========= STATE: AnyQuestions =========
	# Detect hands
	# If no hands detected, update state AnyQuestions
	if NO_HANDS_DETECTED:
		Info.Send("State", {"AnyQuestions": "NoHandsRaised"})

	# Else if hands detected, update state, then send hands info to kinematics
	else:
		Info.Send("State", {"AnyQuestions": "HandsRaised"})
		for hand_info in list_of_hands_info:
			Info.Send("RaisedHandInfo", {
				"bounding_box": (x,y,w,h),
				"frame_res": (1920,1080),
				"confidence_score": -1.0, # send float, if not there'll be an error
				"number": len(list_of_hands_info)
			}

	# Continue when no hands detected or QnA loop ended
	# (In which case, Kinematics module will update state AnyQuestions to "NoHandsRaised" once all questions have been answered)


	# ========= STATE: NoiseLevel =========
	# Wait for update on state change
	state_noise_level = Info.Request("State", {"name": "NoiseLevel"})
	while not state_noise_level:
		state_noise_level = Info.Request("State", {"name": "NoiseLevel"})
		time.sleep(.5)

	# If high noise level, robot makes a joke and loop restarts
	if state_noise_level = "High":
		return


	# ========= STATE: Attentiveness =========
	# Start attentiveness detection and calculate total engagement score

	# Update state Attentiveness
	# If not attentive, update state, then control will trigger joke or quiz and loop restarts
	if total_engagement_score < threshold:
		Info.Send("State", {"Attentiveness": "NotAttentive"})

	# If attentive, update state 
	else:
		Info.Send("State", {"Attentiveness": "Attentive"})


	# ========= STATE: NoQuestionsLoop =========
	# nothing to be done
	return	


if __name__ == "__main__":
	PepperAPI.init("cv_node")
	while True:
		cv_main()
