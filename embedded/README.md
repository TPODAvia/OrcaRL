To read Arduino data in Ubuntu terminal, you need to follow these steps:

1. **Install the necessary packages**: You need to install `screen` package which provides a full-screen window terminal mode. It allows you to detach from a running screen session and then reattach later. You can install it using the following command:
   ```bash
   sudo apt-get install screen
   ```

2. **Identify the USB Port**: Identify the USB port that your Arduino is connected to. You can do this by typing `ls /dev/tty*` in the terminal. This will list all the tty devices. Your Arduino should be listed as one of them, usually `/dev/ttyACM0` or `/dev/ttyUSB0`.

3. **Connect to the Arduino via Terminal**: Now, you can connect to the Arduino via terminal using the `screen` command followed by the device name. For example, if your Arduino is connected to `/dev/ttyACM0`, you would type:
   ```bash
   screen /dev/ttyACM0 115200
   ```
  Here, `115200` is the baud rate set in your Arduino code.

4. **Reading the Data**: Once you're connected, any data sent from the Arduino over the serial connection will appear in the terminal. In your case, you should see "hello world" printed every second.

Remember to press `Ctrl+A` followed by `D` to detach from the screen session when you're done reading data. You can reattach to the session later using `screen -r`.

Please note that the above instructions assume that you have already installed the Arduino IDE on your Ubuntu machine and uploaded the sketch to your Arduino board.


If you want to terminate a `screen` session, you can use the following commands:

1. **Detaching from the Screen Session**: If you just want to temporarily disconnect from the `screen` session (but not end it), you can use `Ctrl + A` followed by `D`. This detaches you from the session, but leaves it running in the background.

2. **Killing the Screen Session**: To completely stop a `screen` session, you first need to know its session ID. You can get a list of all active `screen` sessions by running:
  ```bash
  screen -ls
  ```
  This will output something like:
  ```bash
   There are screens on:
       12345.example (Detached)
       67890.another (Detached)
   2 Sockets in /var/run/screen/S-username.
  ```
  Each line represents an active `screen` session. The number at the beginning of each line is the session ID.

3. **Terminating the Screen Session**: Once you have the session ID, you can terminate the session with the `-X` option followed by `quit`:
  ```bash
  screen -X -S 12345 quit
  ```
  Replace `12345` with your actual session ID. This will terminate the `screen` session with the specified ID.