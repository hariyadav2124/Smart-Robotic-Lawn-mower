#!/bin/bash

# 1. Folder mein jayein
cd /home/spiderman/ggh || { echo "❌ Folder nahi mila"; exit 1; }

# 2. Virtual Environment activate karein
echo "🔄 Activating environment..."
source /home/spiderman/gg/bin/activate

# 3. Robot ko start karein
echo "🤖 Starting GrassGuard AI..."
python3 brain.py
