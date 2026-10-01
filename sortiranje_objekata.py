import argparse
import msvcrt
import time
from pathlib import Path

import cv2
import numpy as np
import requests


OBJECTS = {
    "1": ("bijeli krug", "white", "circle"),
    "2": ("plavi krug", "blue", "circle"),
    "3": ("crveni pravokutnik", "red", "rectangle"),
    "4": ("bijeli pravokutnik", "white", "rectangle"),
}

COLOR_RANGES = {
    "white": [(np.array([0, 0, 155]), np.array([180, 80, 255]))],
    "blue": [(np.array([90, 45, 45]), np.array([129, 255, 255]))],
    "red": [
        (np.array([0, 55, 60]), np.array([12, 255, 255])),
        (np.array([168, 55, 60]), np.array([180, 255, 255])),
    ],
}

MIN_OBJECT_AREA = 250
OUTPUT_DIR = Path(__file__).with_name("sorting_output")
SERVO_RETURN_WAIT_SECONDS = 5.0
HTTP_TIMEOUT_SECONDS = 120


def normalize_url(value):
    value = value.strip().rstrip("/")
    if not value.startswith(("http://", "https://")):
        value = "http://" + value
    return value


def request_text(esp32_url, route, timeout=HTTP_TIMEOUT_SECONDS):
    response = requests.get(esp32_url + route, timeout=timeout)
    response.raise_for_status()
    return response.text.strip()


def read_events(esp32_url):
    response = requests.get(
        esp32_url + "/events", timeout=HTTP_TIMEOUT_SECONDS
    )
    response.raise_for_status()
    return response.json()


def capture_frame(esp32_url):
    response = requests.get(
        esp32_url + "/capture", timeout=HTTP_TIMEOUT_SECONDS
    )
    response.raise_for_status()
    image_data = np.frombuffer(response.content, dtype=np.uint8)
    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("Primljena slika se ne moze procitati.")
    return frame


def choose_target():
    print("\nOdaberite objekt koji zelite sortirati:")
    for number, (label, _, _) in OBJECTS.items():
        print(f"  {number}. {label}")

    while True:
        choice = input("Unesite broj 1-4: ").strip()
        if choice in OBJECTS:
            return OBJECTS[choice]
        print("Neispravan izbor. Unesite broj od 1 do 4.")


def select_analysis_roi(frame):
    print("\nOznacite podrucje u kojem se objekt zaustavlja kod IR1.")
    print("Misem povucite pravokutnik, zatim pritisnite Enter ili Space.")

    while True:
        x, y, width, height = cv2.selectROI(
            "Oznacite ROI za analizu", frame, showCrosshair=True, fromCenter=False
        )
        cv2.destroyWindow("Oznacite ROI za analizu")
        if width >= 20 and height >= 20:
            print(f"ROI spremljen: x={x}, y={y}, sirina={width}, visina={height}")
            return int(x), int(y), int(width), int(height)
        print("ROI nije oznacen. Pokusajte ponovno.")


def color_mask(roi, color):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def largest_object(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [c for c in contours if cv2.contourArea(c) >= MIN_OBJECT_AREA]
    if not candidates:
        return None

    height, width = mask.shape[:2]
    margin = 3
    inside = []
    for contour in candidates:
        x, y, contour_width, contour_height = cv2.boundingRect(contour)
        touches_edge = (
            x <= margin
            or y <= margin
            or x + contour_width >= width - margin
            or y + contour_height >= height - margin
        )
        if not touches_edge:
            inside.append(contour)

    usable = inside if inside else candidates
    center_x = width / 2.0
    center_y = height / 2.0

    def contour_score(contour):
        area = cv2.contourArea(contour)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return 0
        object_x = moments["m10"] / moments["m00"]
        object_y = moments["m01"] / moments["m00"]
        distance = np.hypot(object_x - center_x, object_y - center_y)
        return area / (1.0 + 0.02 * distance)

    return max(usable, key=contour_score)


def identify_shape(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return "unknown"

    x, y, width, height = cv2.boundingRect(contour)
    aspect = width / float(height) if height else 0
    circularity = 4.0 * np.pi * area / (perimeter * perimeter)
    rectangle_fill = area / float(width * height) if width and height else 0

    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    hull_perimeter = cv2.arcLength(hull, True)
    solidity = area / hull_area if hull_area > 0 else 0
    approximation = cv2.approxPolyDP(hull, 0.04 * hull_perimeter, True)

    if (
        len(approximation) == 4
        and rectangle_fill >= 0.65
        and solidity >= 0.85
    ):
        return "rectangle"
    if (
        len(approximation) >= 5
        and circularity >= 0.55
        and 0.60 <= aspect <= 1.65
    ):
        return "circle"
    return "unknown"


def analyze(frame, roi_rect):
    offset_x, offset_y, width, height = roi_rect
    roi = frame[offset_y : offset_y + height, offset_x : offset_x + width]
    best = None

    for color in ("white", "blue", "red"):
        mask = color_mask(roi, color)
        contour = largest_object(mask)
        if contour is None:
            continue
        area = cv2.contourArea(contour)
        if best is None or area > best[0]:
            best = (area, color, contour, mask)

    preview = frame.copy()
    cv2.rectangle(
        preview,
        (offset_x, offset_y),
        (offset_x + width, offset_y + height),
        (255, 0, 0),
        2,
    )

    if best is None:
        return "unknown", "unknown", preview, np.zeros(roi.shape[:2], np.uint8)

    _, color, contour, mask = best
    shape = identify_shape(contour)
    full_contour = contour.copy()
    full_contour[:, :, 0] += offset_x
    full_contour[:, :, 1] += offset_y
    cv2.drawContours(preview, [full_contour], -1, (0, 255, 0), 2)
    cv2.putText(
        preview,
        f"{color} {shape}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2,
    )
    return color, shape, preview, mask


def send_decision(esp32_url, is_target):
    value = 1 if is_target else 0
    request_text(esp32_url, f"/decision?target={value}")


def wait_for_start():
    input("\nKada ste spremni, pritisnite Enter za pokretanje trake.")


def keyboard_action():
    key = None
    if msvcrt.kbhit():
        key = msvcrt.getwch().lower()

    window_key = cv2.waitKey(1) & 0xFF
    if window_key in (ord("q"), ord("o")):
        key = chr(window_key)

    if key == "q":
        return "quit"
    if key == "o":
        return "change"
    return None


def wait_with_keyboard(seconds):
    end_time = time.monotonic() + seconds
    while time.monotonic() < end_time:
        action = keyboard_action()
        if action:
            return action
        time.sleep(0.05)
    return None


def change_target(esp32_url):
    request_text(esp32_url, "/motor_off")
    send_decision(esp32_url, False)
    request_text(esp32_url, "/servo_center")
    print("\nTraka je zaustavljena radi promjene objekta interesa.")

    selected = choose_target()
    print(f"Odabrali ste: {selected[0]}.")
    input("Pritisnite Enter za nastavak rada.")

    read_events(esp32_url)
    request_text(esp32_url, "/motor_on")
    print("Traka je ponovno pokrenuta.")
    return selected


def wait_for_ir2_cycle(esp32_url, is_target):
    print("Cekam da objekt stigne do IR2. q = kraj, o = promjena objekta.")

    while True:
        action = keyboard_action()
        if action:
            return action
        events = read_events(esp32_url)
        if events.get("ir2_state") == 0:
            break
        time.sleep(0.05)

    print("Objekt je stigao do IR2.")

    if is_target:
        action = wait_with_keyboard(0.15)
        if action:
            return action
        request_text(esp32_url, "/servo_left")
        print("Servo izdvaja objekt i ostaje u tom polozaju 5 sekundi.")
        action = wait_with_keyboard(SERVO_RETURN_WAIT_SECONDS)
        request_text(esp32_url, "/servo_center")
        if action:
            return action
        print("Servo se vratio u sredinu.")

    while True:
        action = keyboard_action()
        if action:
            return action
        events = read_events(esp32_url)
        if events.get("ir2_state") != 0:
            break
        time.sleep(0.05)

    print("Sustav je spreman za novi objekt.")
    return "done"


def save_analysis(frame, mask, object_number):
    OUTPUT_DIR.mkdir(exist_ok=True)
    cv2.imwrite(str(OUTPUT_DIR / f"objekt_{object_number:03d}.jpg"), frame)
    cv2.imwrite(str(OUTPUT_DIR / f"maska_{object_number:03d}.png"), mask)


def main():
    parser = argparse.ArgumentParser(description="ESP32-CAM sortiranje objekata")
    parser.add_argument("--esp32", help="Primjer: 192.168.1.50")
    args = parser.parse_args()

    address = args.esp32 or input("Unesite IP adresu ESP32-CAM: ")
    esp32_url = normalize_url(address)

    print("Povezivanje s kamerom...")
    first_frame = capture_frame(esp32_url)
    print("Kamera je povezana.")

    input("Ostavite traku praznu i pritisnite Enter za odabir ROI podrucja.")
    roi_frame = capture_frame(esp32_url)
    roi_rect = select_analysis_roi(roi_frame)

    target_label, target_color, target_shape = choose_target()
    print(f"Odabrali ste: {target_label}.")
    wait_for_start()

    read_events(esp32_url)
    request_text(esp32_url, "/servo_center")
    request_text(esp32_url, "/motor_on")
    print("Traka je pokrenuta. q = kraj programa, o = promjena objekta interesa.")

    object_number = 0
    try:
        while True:
            try:
                action = keyboard_action()
                if action == "quit":
                    break
                if action == "change":
                    target_label, target_color, target_shape = change_target(esp32_url)
                    continue

                events = read_events(esp32_url)
                ir1_state = events.get("ir1_state")
                if events.get("ir1") != 1 and ir1_state != 0:
                    time.sleep(0.05)
                    continue

                object_number += 1
                print(f"\nObjekt {object_number}: IR1 aktiviran, zaustavljam traku.")
                request_text(esp32_url, "/motor_off")
                time.sleep(0.2)

                frame = capture_frame(esp32_url)
                color, shape, preview, mask = analyze(frame, roi_rect)
                is_target = color == target_color and shape == target_shape
                print(f"Prepoznato: {color} {shape}")
                print("Odluka:", "IZDVOJI" if is_target else "PROPUSTI")

                send_decision(esp32_url, False)
                save_analysis(preview, mask, object_number)
                cv2.imshow("ESP32-CAM analiza", preview)
                cv2.imshow("Maska objekta", mask)
                cv2.waitKey(1)

                request_text(esp32_url, "/motor_on")
                print("Traka je ponovno pokrenuta.")

                cycle_result = wait_for_ir2_cycle(esp32_url, is_target)
                if cycle_result == "quit":
                    break
                if cycle_result == "change":
                    target_label, target_color, target_shape = change_target(esp32_url)

            except requests.RequestException as error:
                print(f"Greska u komunikaciji: {error}")
                time.sleep(0.5)
            except Exception as error:
                print(f"Greska pri obradi objekta: {error}")
                try:
                    send_decision(esp32_url, False)
                    request_text(esp32_url, "/motor_on")
                except requests.RequestException:
                    pass
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nProgram se zaustavlja.")
    finally:
        try:
            request_text(esp32_url, "/motor_off")
            request_text(esp32_url, "/servo_center")
        except requests.RequestException:
            pass
        cv2.destroyAllWindows()
        print(f"Detektirano ukupno {object_number} objekata.")


if __name__ == "__main__":
    main()
