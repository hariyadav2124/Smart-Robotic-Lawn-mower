#!/bin/bash
# GrassGuard AI — One-line launcher

echo "=== GrassGuard AI Starting ==="

# Purane process band karo
pkill -f brain.py 2>/dev/null
pkill -f d5pi.py 2>/dev/null
sleep 2
echo "✅ Purane process band"

# Venv activate + ggh folder
source /home/spiderman/gg/bin/activate
cd /home/spiderman/ggh
echo "✅ Venv activated, ggh folder mein"

# Brain start
echo "🚀 Brain starting..."
echo ""
python3 brain.py
