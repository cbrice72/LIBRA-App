 # Setup for LIBRA_App on WSL2

## Proxy Configuration

In `/etc/apt/apt.conf` (requires sudo):
```txt
Acquire::http::Proxy "http://proxy.noc.titech.ac.jp:3128";
```

## Qt

### *Install Dependencies*

Try this first:
```bash
sudo apt install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev
```

If Qt still doesn't run after the steps below, install each of the following packages one by one:
- `libfontconfig`
- `libxcb-glx0`
- `libx11-xcb1`
- `libxcb-icccm4`
- `libxcb-image0`
- `libxcb-keysyms1`
- `libxcb-shape0`
- `libxcb-xkb1`
- `libxcb-xinerama0`
- `libxkbcommon-x11-0`
- `libegl1`

```bash
sudo apt install -y libfontconfig libxcb-glx0 libx11-xcb1 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-shape0 libxcb-xkb1 libxcb-xinerama0 libxkbcommon-x11-0 libegl1
```

> **_NOTE:_** Might be missing some, will have to check on laptop.

### *Install Qt*

```bash
chmod +x qt-unified-linux-x64-[ver]-online.run
./qt-unified-linux-x64-[ver]-online.run
```

> **_NOTE:_** To set the proxy, open the settings menu (bottom left) and select "Manual proxy configuration".
Enter HTTP proxy: `proxy.noc.titech.ac.jp` Port: `3128`.

### *Add QtCreator to PATH*

In `~/.bashrc`:
```bash
if [ -d "$HOME/Qt/Tools" ]; then
    PATH="$PATH:$HOME/Qt/Tools/QtCreator/bin"
fi
```
