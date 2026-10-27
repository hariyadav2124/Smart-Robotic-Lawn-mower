#!/usr/bin/env python3
"""
GrassGuard brain.py patch script — P0 through P8.
Each patch must match exactly once; abort before write if any target drifts.
"""
import os, sys, time, shutil, py_compile

if not os.path.isfile('brain.py'):
    sys.exit('brain.py not found in current directory')

BACKUP = 'brain.py.pre_patch_%s' % time.strftime('%Y%m%d_%H%M%S')
shutil.copy2('brain.py', BACKUP)
print('Backup ->', BACKUP)

with open('brain.py', encoding='utf-8') as f:
    src = f.read()

PATCHES = []

# ─── P0: BOXED_BACK can actually back up ────────────────────────────────
PATCHES.append((
    'P0a: brain_send force kwarg',
    "def brain_send(c, wait=0.15):",
    "def brain_send(c, wait=0.15, force=False):",
))

PATCHES.append((
    'P0b: guard skipped when force=True',
    "    if c == 'B':\n        b_val = float(T['b']) if is_valid_distance(T['b']) else 999.0",
    "    if c == 'B' and not force:\n        b_val = float(T['b']) if is_valid_distance(T['b']) else 999.0",
))

PATCHES.append((
    'P0c: BOXED_BACK loop passes force=True',
    "                        if b_chk <= 10.0:\n"
    "                            log('BACK LIMIT REACHED: B:%.0fcm - STOP' % b_chk)\n"
    "                            break\n"
    "                        brain_send('B', 0.1)",
    "                        if b_chk <= 10.0:\n"
    "                            log('BACK LIMIT REACHED: B:%.0fcm - STOP' % b_chk)\n"
    "                            break\n"
    "                        brain_send('B', 0.1, force=True)",
))

# ─── P1: remove 2a blade-off flap caused by cam-US vs person ────────────
PATCHES.append((
    'P1: delete section 2a (blade flap)',
    "        # ---- 2a. LIVING CLOSE -> blade OFF (only if <=50cm) ----\n"
    "        if name in LIVING and S['blade'] == 'ON':\n"
    "            pd_2a = T['c'] if (time.time() - ARD_ST['age'] < 3.0 and is_valid_distance(T['c']) and T['c'] < 900) else T['f']\n"
    "            if pd_2a <= SOFTF_CM:  # Only if within 50cm\n"
    "                brain_send('K'); S['blade'] = 'CUT'\n"
    "                log('%s CLOSE (%.0fcm) - BLADE OFF (safety)' % (name.upper(), pd_2a))",
    "        # ---- 2a. (REMOVED: section 3 handles person-close, zone-aware) ----",
))

# ─── P2a: init flow_last_close ─────────────────────────────────────────
PATCHES.append((
    'P2a: init flow_last_close',
    "    person_close_at = 0.0\n    acs_lock_until = 0.0",
    "    person_close_at = 0.0\n    flow_last_close = 0.0\n    acs_lock_until = 0.0",
))

# ─── P2c: correct resume-log + 3s grace before state reset ─────────────
PATCHES.append((
    'P2c: resume-log uses flow_last_close + 3s grace on state reset',
    "            if pd > 55.0:\n"
    "                if state == 'LIVING_FLOW':\n"
    "                    # Sirf tab resume bolo jab 3 second ho chuke hon\n"
    "                    if now - person_close_at > 3.0:\n"
    "                        log('%s door ho gaya (%.0fcm) - resume' % (name.upper(), pd))\n"
    "                        try: tts_speak('resume_mow')\n"
    "                        except: pass\n"
    "                state = ''\n",
    "            if pd > 55.0:\n"
    "                if state == 'LIVING_FLOW':\n"
    "                    # Sirf tab resume bolo jab 3 second ho chuke hon\n"
    "                    if now - flow_last_close > 3.0:\n"
    "                        log('%s door ho gaya (%.0fcm) - resume' % (name.upper(), pd))\n"
    "                        try: tts_speak('resume_mow')\n"
    "                        except: pass\n"
    "                        state = ''\n"
    "                    # else: 3s grace period — stay in LIVING_FLOW\n"
    "                else:\n"
    "                    state = ''\n",
))

# ─── P2d: track flow_last_close when person is close ───────────────────
PATCHES.append((
    'P2d: track flow_last_close when person is close',
    "            else:\n                # Blade OFF\n                if S['blade'] == 'ON':",
    "            else:\n                flow_last_close = now\n                # Blade OFF\n                if S['blade'] == 'ON':",
))

# ─── P6: remove orphan continue ────────────────────────────────────────
PATCHES.append((
    'P6: remove orphan continue',
    "                    state = ''\n"
    "                    time.sleep(0.5)\n"
    "                    continue\n"
    "                    continue\n",
    "                    state = ''\n"
    "                    time.sleep(0.5)\n"
    "                    continue\n"
))

# ─── P2e: add LIVING_FLOW FOV-clear ────────────────────────────────────
PATCHES.append((
    'P2e: add LIVING_FLOW FOV-clear',
    "        # ---- 3c9. SIDE STUCK - F + L/R <50cm -> back 0.5s + redirect ----",
    "        elif state == 'LIVING_FLOW':\n"
    "            # person gone from camera entirely — clear flow state after grace\n"
    "            if now - flow_last_close > 3.0:\n"
    "                state = ''\n"
    "\n"
    "        # ---- 3c9. SIDE STUCK - F + L/R <50cm -> back 0.5s + redirect ----"
))

# ─── P3: clear_t as real elapsed time (not loop-iteration count) ────────
PATCHES.append((
    'P3a: clear_t tracks timestamp, not ticks',
    "        sab_saaf = (kyun == '')\n"
    "        if sab_saaf:\n"
    "            clear_t += 0.3\n"
    "        else:\n"
    "            clear_t = 0.0",
    "        sab_saaf = (kyun == '')\n"
    "        if sab_saaf:\n"
    "            if clear_t == 0.0:\n"
    "                clear_t = now\n"
    "        else:\n"
    "            clear_t = 0.0",
))

PATCHES.append((
    'P3b: blade-ON check uses real elapsed time',
    "        if (S['blade'] == 'CUT' and MODE in ('auto', 'mow') and sab_saaf\n"
    "                and clear_t >= BLADE_CLEAR_SEC and now >= acs_lock_until):\n"
    "            brain_send('M'); S['blade'] = 'ON'\n"
    "            log('SAB SAAF %.1fs — blade KHUD ON' % clear_t)",
    "        if (S['blade'] == 'CUT' and MODE in ('auto', 'mow') and sab_saaf\n"
    "                and clear_t > 0.0 and now - clear_t >= BLADE_CLEAR_SEC and now >= acs_lock_until):\n"
    "            brain_send('M'); S['blade'] = 'ON'\n"
    "            log('SAB SAAF %.1fs — blade KHUD ON' % (now - clear_t))",
))

# ─── P4: validate raw T['f'] before 999 fallback ──────────────────────
PATCHES.append((
    'P4: validate raw front sensor before 999 fallback',
    "        # Front detection: F + C dono use karo (backup)\n"
    "        f_tmp = float(T['f']) if is_valid_distance(T['f']) else 999.0\n"
    "        c_tmp = float(T['c']) if (time.time() - ARD_ST['age'] < 3.0 and is_valid_distance(T['c']) and float(T['c']) < 900) else 999.0\n"
    "        fcm = min(f_tmp, c_tmp)  # Jo bhi paas ho (F ya C)\n"
    "        lcm, rcm = T['l'], T['r']\n"
    "        # now = time.time()  # moved above\n"
    "\n"
    "        # Sensor validity guard: NaN or invalid distances must never authorize movement\n"
    "        if (not is_valid_distance(fcm) or not is_valid_distance(lcm)\n"
    "                or not is_valid_distance(rcm)):",
    "        # Raw sensor validity check BEFORE 999 fallback (catches broken sensors)\n"
    "        lcm, rcm = T['l'], T['r']\n"
    "        if (not is_valid_distance(T['f']) or not is_valid_distance(lcm)\n"
    "                or not is_valid_distance(rcm)):",
))

PATCHES.append((
    'P4b: re-add fcm computation after the guard',
    "            brain_send('K'); S['blade'] = 'CUT'; brain_send('S')\n"
    "            time.sleep(0.4); continue\n"
    "\n"
    "        # ---- 2a.",
    "            brain_send('K'); S['blade'] = 'CUT'; brain_send('S')\n"
    "            time.sleep(0.4); continue\n"
    "\n"
    "        f_tmp = float(T['f']) if is_valid_distance(T['f']) else 999.0\n"
    "        c_tmp = float(T['c']) if (time.time() - ARD_ST['age'] < 3.0 and is_valid_distance(T['c']) and float(T['c']) < 900) else 999.0\n"
    "        fcm = min(f_tmp, c_tmp)  # Jo bhi paas ho (F ya C)\n"
    "\n"
    "        # ---- 2a.",
))

# ─── P7a: rectangle validator ─────────────────────────────────────────
PATCHES.append((
    'P7a: add mow_validate_corners()',
    "    return None\n\ndef get_local_ip():",
    "    return None\n"
    "\n"
    "def mow_validate_corners():\n"
    "    c = MOWW['corners']\n"
    "    if len(c) < 4:\n"
    "        return 'corners %d/4 set' % len(c)\n"
    "    try:\n"
    "        A, B, C, D = c[1], c[2], c[3], c[4]\n"
    "    except KeyError as e:\n"
    "        return 'corner %s missing' % e\n"
    "    AB = hav_m(A, B); BC = hav_m(B, C); CD = hav_m(C, D); DA = hav_m(D, A)\n"
    "    AC = hav_m(A, C); BD = hav_m(B, D)\n"
    "    if min(AB, BC, CD, DA) < 0.5:\n"
    "        return 'koi side 0.5m se chhoti — points galat'\n"
    "    tol = 0.25\n"
    "    if abs(AB - CD) > tol * max(AB, CD):\n"
    "        return 'sides 1-2 aur 3-4 alag (%.1fm vs %.1fm) — order 1,2,3,4 galat' % (AB, CD)\n"
    "    if abs(BC - DA) > tol * max(BC, DA):\n"
    "        return 'sides 2-3 aur 4-1 alag (%.1fm vs %.1fm) — order galat' % (BC, DA)\n"
    "    if abs(AC - BD) > tol * max(AC, BD):\n"
    "        return 'diagonals alag (%.1fm vs %.1fm) — order galat' % (AC, BD)\n"
    "    return 'OK'\n"
    "\n"
    "def get_local_ip():",
))

# ─── P5 + P7b + P8: /mow?a=start: validate + stale warn + '0'→'1' ─────
PATCHES.append((
    'P5+P7b+P8: rewrite /mow start block',
    "    if a == 'start':\n"
    "        try:\n"
    "            sp = float(request.args.get('sp', '1.0'))\n"
    "        except Exception:\n"
    "            sp = 1.0\n"
    "        sp = max(0.4, min(3.0, sp))\n"
    "        MOWW['spacing'] = sp\n"
    "        n = mow_build()\n"
    "        if n == 0:\n"
    "            return 'pehle 4 CORNER set karo!'\n"
    "        MODE = 'mow'\n"
    "        S['mode'] = 'mow'\n"
    "        del GPS_HIST[:]\n"
    "        brain_send('S'); brain_send('0')   # TEST: 20% slow speed\n"
    "        ard_send('A'); esp_send('A')\n"
    "        log('MOW START: %d lines, spacing %.1fm — ZIGZAG chalu!' % (n, sp))\n"
    "        return 'MOW START (%d lines, %.1fm)' % (n, sp)",
    "    if a == 'start':\n"
    "        v = mow_validate_corners()\n"
    "        if v != 'OK':\n"
    "            log('MOW START REJECT: ' + v)\n"
    "            return 'CORNERS GALAT: ' + v\n"
    "        try:\n"
    "            sp = float(request.args.get('sp', '1.0'))\n"
    "        except Exception:\n"
    "            sp = 1.0\n"
    "        sp = max(0.4, min(3.0, sp))\n"
    "        MOWW['spacing'] = sp\n"
    "        stale = []\n"
    "        cts = MOWW.get('corner_ts', {})\n"
    "        for j in (1, 2, 3, 4):\n"
    "            ts = cts.get(j, 0)\n"
    "            if ts and time.time() - ts > 60:\n"
    "                stale.append('C%d(%.0fs)' % (j, time.time() - ts))\n"
    "        if stale:\n"
    "            log('MOW START WARN: purane corners ' + ', '.join(stale))\n"
    "        n = mow_build()\n"
    "        if n == 0:\n"
    "            return 'pehle 4 CORNER set karo!'\n"
    "        MODE = 'mow'\n"
    "        S['mode'] = 'mow'\n"
    "        del GPS_HIST[:]\n"
    "        brain_send('S'); brain_send('1')   # '1' = slowest defined in SPDL\n"
    "        ard_send('A'); esp_send('A')\n"
    "        log('MOW START: %d lines, spacing %.1fm — ZIGZAG chalu!' % (n, sp))\n"
    "        return 'MOW START (%d lines, %.1fm)%s' % (n, sp, (' [WARN stale: ' + ', '.join(stale) + ']') if stale else '')",
))

# ─── P8b: /corner records timestamp ───────────────────────────────────
PATCHES.append((
    'P8b: /corner records timestamp',
    "    MOWW['corners'][i] = (glat, glon)\n"
    "    log('MOW: CORNER %d set (%s, acc %.0fm)' % (i, G['lat'], acc))",
    "    MOWW['corners'][i] = (glat, glon)\n"
    "    MOWW.setdefault('corner_ts', {})[i] = time.time()\n"
    "    log('MOW: CORNER %d set (%s, acc %.0fm)' % (i, G['lat'], acc))",
))

# ─── Apply (unique-match, abort-before-write) ─────────────────────────
ok = []
for name, old, new in PATCHES:
    if old not in src:
        print('\n❌ ABORT — patch [%s] target not found.' % name)
        print('   First 6 lines of target (indent shown with |):')
        for ln in old.split('\n')[:6]:
            print('   | ' + repr(ln))
        sys.exit(1)
    if src.count(old) > 1:
        print('\n❌ ABORT — patch [%s] target appears %d times (must be unique).' % (name, src.count(old)))
        for ln in old.split('\n')[:6]:
            print('   | ' + repr(ln))
        sys.exit(1)
    src = src.replace(old, new, 1)
    ok.append(name)
    print('  OK  %s' % name)

with open('brain.py', 'w', encoding='utf-8') as f:
    f.write(src)

try:
    py_compile.compile('brain.py', doraise=True)
    print('\n✅ COMPILE OK — %d patches applied.' % len(ok))
    print('   Backup: %s' % BACKUP)
    print('   Undo  : cp %s brain.py' % BACKUP)
    print('\nNext: source ~/gg/bin/activate && cd ~/ggh && python brain.py')
except py_compile.PyCompileError as e:
    print('\n❌ COMPILE FAILED: ' + str(e))
    shutil.copy2(BACKUP, 'brain.py')
    print('   Reverted to %s' % BACKUP)
    sys.exit(1)
