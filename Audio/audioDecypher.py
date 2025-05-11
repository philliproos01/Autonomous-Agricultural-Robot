import speech_recognition as sr
import time

recognizer = sr.Recognizer()
mic = sr.Microphone()

WAKE_WORD = "ok robot"
STOP_WORD = "stop responding"
SILENCE_TIMEOUT = 5  # seconds

print("Listening for wake word... (say 'ok robot' to start recording)")
print(f"Say '{STOP_WORD}' at any time to deactivate the robot.")

running = True
while running:
    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = recognizer.listen(source)
    try:
        transcript = recognizer.recognize_google(audio).lower()
        print(f"Heard: {transcript}")
        # Check for stop command first
        if STOP_WORD in transcript:
            print("Deactivation phrase detected. Robot is deactivating. Goodbye!")
            break
        # Check for wake word
        elif WAKE_WORD in transcript:
            print("Wake word detected! Start speaking. Pause for 5 seconds to finish.")
            collected_text = []
            last_speech_time = time.time()
            while True:
                with mic as source:
                    recognizer.adjust_for_ambient_noise(source)
                    try:
                        # Listen for up to 5 seconds of silence
                        audio = recognizer.listen(source, timeout=SILENCE_TIMEOUT, phrase_time_limit=10)
                        phrase = recognizer.recognize_google(audio)
                        print(f"Recognized: {phrase}")
                        # Check for stop command in the phrase
                        if STOP_WORD in phrase.lower():
                            print("Deactivation phrase detected. Robot is deactivating. Goodbye!")
                            running = False
                            break
                        collected_text.append(phrase)
                        last_speech_time = time.time()
                    except sr.WaitTimeoutError:
                        # No speech detected within the silence timeout
                        print("Silence detected. Stopping recording.")
                        break
                    except sr.UnknownValueError:
                        print("Could not understand audio, continue listening...")
                    except sr.RequestError as e:
                        print(f"Could not request results; {e}")
                        break
            final_text = " ".join(collected_text)
            print(f"Final recorded command: {final_text}")
            with open("robot_command.txt", "w") as f:
                f.write(final_text)
            print("Command saved to robot_command.txt")
            if not running:
                break
    except sr.UnknownValueError:
        print("Did not catch that. Listening again...")
    except sr.RequestError as e:
        print(f"Could not request results; {e}")
