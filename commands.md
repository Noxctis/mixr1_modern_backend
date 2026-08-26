# Without FIFO (normal scheduler)
sudo ./mixr1_daemon --test --sweep --no-fifo --no-pi \
  --duration=10 --target=0 --csv=raw_no_fifo.csv //did this

# With FIFO (real-time scheduler)
sudo ./mixr1_daemon --test --sweep --fifo --no-pi \
  --duration=10 --target=0 --csv=raw_fifo.csv //did this




# Without FIFO (normal scheduler)
sudo ./mixr1_daemon --test --sweep --no-fifo --pi \
  --target=2500 --duration=10 --csv=pi_no_fifo.csv //did this

# With FIFO (real-time scheduler)
sudo ./mixr1_daemon --test --sweep --fifo --pi \
  --target=2500 --duration=10 --csv=pi_fifo.csv //did this




# Open-loop with FIFO
sudo ./mixr1_daemon --test --sweep --fifo --no-pi \
  --duration=10 --target=0 --csv=water_raw_fifo.csv

# PI with FIFO
sudo ./mixr1_daemon --test --sweep --fifo --pi \
  --target=2500 --duration=10 --csv=water_pi_fifo.csv




# Analyze FIFO runs (air, no water load)
python3 analyze_pwm_sweep.py \
  raw_fifo.csv pi_fifo.csv \
  --settle=2 \
  --settling-band=0.05 \
  --output=fifo_analysis

# Analyze water-loaded runs
python3 analyze_pwm_sweep.py \
  water_raw_fifo.csv water_pi_fifo.csv \
  --settle=2 \
  --settling-band=0.05 \
  --output=water_analysis




