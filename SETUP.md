 # Setup for LIBRA_App on WSL2

## Proxy Configuration

In `/etc/apt/apt.conf` (requires sudo):
```txt
Acquire::http::Proxy "http://proxy.noc.titech.ac.jp:3128";
```

## Qt

### *Install Dependencies*

```bash
sudo apt install -y ...
```

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