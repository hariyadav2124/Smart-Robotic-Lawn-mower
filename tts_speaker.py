"""
GrassGuard TTS Speaker — English + Non-blocking + Anti-Spam
"""
import subprocess
import threading
import time

MESSAGES = {
    # ---- STARTUP ----
    'startup':          "Hello! GrassGuard AI ready.",
    'boot_complete':    "All sensors ready.",

    # ---- MODES ----
    'mode_auto':        "Auto mode activated.",
    'mode_mow':         "Mow mode ready.",
    'mode_manual':      "Manual mode activated.",
    'mode_stop':        "All systems stopped.",

    # ---- LIVING / PERSON ----
    'person_detected':  "Person detected. Please move aside.",
    'person_ahead':     "Person ahead. Please move aside.",
    'stopping':         "Stopping for safety.",
    'please_move':      "Please move away.",

    # ---- BLADE ----
    'blade_on':         "Blade active.",
    'blade_off':        "Blade stopped.",

    # ---- AVOIDANCE ----
    'avoid_mode':       "Avoiding obstacle.",
    'back_track':       "Obstacle ahead. Finding route.",
    'resume_mow':       "Path clear. Resuming.",
    'forward_clear':    "Path clear.",

    # ---- SAFETY ----
    'rain_stop':        "Rain detected. Stopping.",
    'tilt_stop':        "Tilt detected. Stopping.",
    'battery_low':      "Battery low.",
    'error_stop':       "Error detected. Stopping.",

    # ---- REAR GUARD ----
    'rear_detected':    "Object behind.",
    'rear_stop':        "Rear blocked. Stopping.",
    'rear_clear':       "Rear clear.",
    'rear_backing':     "Moving backward.",

    # ---- COMPLETE ----
    'mow_complete':     "Mowing complete.",
    'welcome':          "GrassGuard AI ready.",
}

_last_spoken_time = {}       # key -> timestamp
_global_last_spoken = 0.0    # timestamp of ANY message
_GLOBAL_COOLDOWN = 3.5       # At least 3.5s between ANY two messages (unless force)
_KEY_COOLDOWN = 8.0          # Same key cannot repeat within 8.0s
_current_proc = None         # Track current subprocess.Popen
_proc_lock = threading.Lock()
_busy = False

def speak(key, force=False):
    """
    Speak a predefined message with strict anti-spam and no queue backlog.
    - If TTS is currently speaking and force=False, drops the request (no backlog lag).
    - If force=True, interrupts current speech and speaks immediately.
    - Enforces global cooldown (3.5s) and per-key cooldown (8.0s).
    """
    global _busy, _global_last_spoken, _current_proc
    if key not in MESSAGES:
        return

    now = time.time()
    if not force:
        if _busy:
            return  # Drop if speaker is busy talking
        if now - _global_last_spoken < _GLOBAL_COOLDOWN:
            return  # Drop if any speech happened very recently
        if now - _last_spoken_time.get(key, 0.0) < _KEY_COOLDOWN:
            return  # Drop if this exact message was spoken recently
    else:
        # Force: cancel any ongoing speech process immediately
        with _proc_lock:
            if _current_proc and _current_proc.poll() is None:
                try:
                    _current_proc.terminate()
                except Exception:
                    pass

    _global_last_spoken = now
    _last_spoken_time[key] = now
    msg = MESSAGES[key]
    threading.Thread(target=_speak_worker, args=(msg,), daemon=True).start()

def _speak_worker(msg):
    global _busy, _current_proc
    with _proc_lock:
        _busy = True
        try:
            # -v en-us = English voice
            # -a 200 = Max volume
            # -s 155 = Clear, brisk speech speed
            p = subprocess.Popen(
                ['espeak-ng', '-v', 'en-us', '-s', '155', '-a', '200', msg],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            _current_proc = p
        except Exception as e:
            print(f"[TTS] start fail: {e}")
            _busy = False
            return

    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            p.kill()
        except Exception:
            pass
    except Exception:
        pass
    finally:
        with _proc_lock:
            if _current_proc == p:
                _current_proc = None
            _busy = False
        print(f"[TTS-en] {msg[:70]}")

if __name__ == '__main__':
    print("Testing English TTS anti-spam...")
    speak('startup', force=True)
    time.sleep(3)
    speak('person_ahead', force=True)
    time.sleep(2)
    speak('resume_mow', force=True)
    time.sleep(2)
    print("Done")
