import requests
import json
from PIL import Image
import io
import sys
import cv2

URL = "http://127.0.0.1:8000/process" 

# IMAGE_PATH = r"C:\Users\sty123\Desktop\lab_stuff\wbcd\ty_shirt_half.jpg"  # Image on your local PC
# Note: This path must be the path ON THE REMOTE SERVER where the model is stored

def sender_receiver(IMAGE_PATH):
    # 1. Open the local image
    try:
        with open(IMAGE_PATH, "rb") as f:
            # Prepare the files and data for the POST request
            files = {"image": (IMAGE_PATH, f, "image/jpeg")}
            #data = {"model_path": REMOTE_MODEL_PATH}

            print(f"🚀 Sending {IMAGE_PATH} to {URL}...")
            response = requests.post(URL, files=files, timeout=10)

        # 2. Check if it worked
        if response.status_code == 200:
            print("✅ Success!")

            # A. Receive and Save the Image
            processed_img = Image.open(io.BytesIO(response.content))
            print("the result is saved as cloths_keypoint.png")
            processed_img.save("cloths_keypoint.png")
            # processed_img.show() # Opens the result on your local screen

            # B. Receive and Print the JSON data (Keypoints)
            # We look for the custom header we created in the server
            results_header = response.headers.get("x-results")
            if results_header:
                keypoints = json.loads(results_header)
                print("📍 Keypoints coordinates received:")
                print(json.dumps(keypoints, indent=2))
            return keypoints
        else:
            print(f"❌ Error {response.status_code}: {response.text}")

    except FileNotFoundError:
        print(f"Error: Could not find {IMAGE_PATH}")
    except requests.exceptions.ConnectionError:
        print("❌ Could not connect to the server. Make sure it is running, then try again.")
        sys.exit(1)
    except requests.exceptions.Timeout:
        print("❌ Server did not respond in time.")
        sys.exit(1)
    except requests.exceptions.RequestException as e:
        print(f"❌ Request failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    keypoints = sender_receiver(IMAGE_PATH)
    print(keypoints)