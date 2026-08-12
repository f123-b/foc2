const { contextBridge } = require('electron');

contextBridge.exposeInMainWorld('focDesktop', { isDesktop: true });
