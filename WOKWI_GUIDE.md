# Running the Adaptive LMS Demo on Wokwi (no Arduino needed)

Wokwi is a free Arduino simulator that runs in your browser. It simulates the Uno, the three potentiometers, the reset button, and the Serial Plotter.

## Setup (do this before the presentation, about 5 min)

1. Go to **https://wokwi.com**. Sign in, since you need an account to **save** the project.
2. Under **"Start a new project"**, click **Arduino Uno**. This opens an editor with two tabs on the left: `sketch.ino` and `diagram.json`.
3. **Paste the code:**
   - Open `Adaptive_LMS.ino` from our folder and copy everything.
   - In Wokwi, click the **`sketch.ino`** tab, select all (Ctrl+A), and paste over it.
4. **Paste the circuit:**
   - Open `diagram.json` from our folder and copy everything.
   - In Wokwi, click the **`diagram.json`** tab, select all, and paste over it.
   - The board, three knobs, and a red "Reset filter" button should appear on the right. You can drag them around to tidy the layout. The wires will follow.
5. Click **Save** (or press Ctrl+S) and give it a name like "Adaptive LMS". **Copy the project link** so you can open it straight away on presentation day.

## Running it

1. Click the green **▶ Start Simulation** button.
2. The **Serial Plotter** should open at the bottom automatically, because `diagram.json` tells Wokwi to start in plotter mode. You don't need to set a baud rate.
3. You should see the Noisy, Clean, Output, Residual, Top, and Bottom lines.

## Controls during the demo

| Wokwi part | Pin | What it does | How to use it in Wokwi |
|---|---|---|---|
| Left knob | A0 | Step size μ | Click the knob, then drag it or use the ← → arrow keys |
| Middle knob | A1 | Noise level | Same as above |
| Right knob | A2 | Signal frequency | Same as above |
| Red button | D2 | Reset filter weights | Click it |

The knobs already start at good demo values: μ ≈ 0.05, medium noise, and a 2 Hz sine wave.

## Test this first! (things to check before presenting)

- **Labels in the legend:** Wokwi's docs don't say whether its plotter supports `Label:value` names like the Arduino IDE does. Run it once and check:
  - **If the legend shows the names (Noisy, Output, and so on), you're good.**
  - **If it shows numbers or no names,** you'll just say the colors out loud instead, for example "the blue line is our input, the red line is our output." Note which color is which before you present.
- **The plot:** it shows three signals, Noisy, Clean, and Output, plus fixed lines at +3 and −3. Click the 🗑 icon under the plotter to clear it, and use the red button instead of Stop/Play to restart learning.
- **Speed:** Simulations sometimes run a bit slower than real time. Check the speed % shown near the top of the simulator. If it's below 100%, the filter just takes a little longer to "lock on" after a reset. That's fine. Just give it a few more seconds in the demo.
- **Internet:** Wokwi needs internet. If the room's Wi-Fi is unreliable, use your phone's hotspot.
