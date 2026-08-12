const { app, BrowserWindow, dialog, session } = require('electron');
const path = require('node:path');

let mainWindow;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 980,
    minWidth: 1060,
    minHeight: 720,
    backgroundColor: '#f3f5f7',
    title: 'FOC Studio',
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      preload: path.join(__dirname, 'preload.cjs'),
    },
  });
  mainWindow.loadFile(path.join(__dirname, '..', 'index.html'));
}

app.whenReady().then(() => {
  // Electron requires the main process to approve and select Web Serial ports.
  session.defaultSession.setPermissionCheckHandler((_webContents, permission) => permission === 'serial');
  session.defaultSession.setDevicePermissionHandler((details) => details.deviceType === 'serial');
  session.defaultSession.on('select-serial-port', (event, portList, _webContents, callback) => {
    event.preventDefault();
    if (!portList.length) return callback('');
    const labels = portList.map((port) => port.displayName || port.portName || `USB ${port.vendorId || ''}:${port.productId || ''}`);
    labels.push('取消');
    const selected = dialog.showMessageBoxSync(mainWindow, {
      type: 'question',
      title: '选择 USB CDC 串口',
      message: '请选择 FOC Studio 控制器 USB CDC 设备',
      buttons: labels,
      cancelId: labels.length - 1,
      defaultId: 0,
    });
    callback(selected < portList.length ? portList[selected].portId : '');
  });
  createWindow();
  app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
