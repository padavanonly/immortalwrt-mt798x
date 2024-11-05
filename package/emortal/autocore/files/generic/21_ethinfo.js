'use strict';
'require baseclass';
'require rpc';

var callLuciETHInfo = rpc.declare({
  object: 'luci',
  method: 'getETHInfo',
  expect: { '': {} }
});

var callLuciNetworkDevices = rpc.declare({
  object: 'luci-rpc',
  method: 'getNetworkDevices',
  expect: { '': {} }
});

return L.Class.extend({
  title: _('Ethernet Information'),

  load: function () {
    return Promise.all([
      L.resolveDefault(callLuciETHInfo(), {}),
      L.resolveDefault(callLuciNetworkDevices(), {})
    ]);
  },

  render: function (data) {
    const ethinfo = Array.isArray(data[0].ethinfo) ? data[0].ethinfo : [];
    const netdevs = typeof data[1] === 'object' ? data[1] : {};

    const boxStyle = 'max-width: 100px;';
    const boxHeadStyle =
      'text-align: center;' +
      'font-size:1.1rem;' +
      'font-weight:bold;' +
      'border-radius: 7px 7px 0 0;';
    const boxbodyStyle =
      'border: 1px solid lightgrey;' +
      'border-radius: 0 0 7px 7px;' +
      'display:flex;' +
      'flex-direction: column;' +
      'align-items: center;' +
      'justify-content:center;';
    const iconStyle = 'margin: 5px; width: 40px;';
    const speedStyle = 'font-size:0.8rem; font-weight:bold;';
    const trafficStyle =
      'border-top: 1px solid lightgrey;' + 'font-size:0.8rem;';

    const ports = [];
    let portColor, portDIV, wanPortDIV;
    for (const port of ethinfo) {
      const portName = port.name;
      const portStatus = port.status == 'yes' ? 'up' : 'down';
      const icon = L.resource(`icons/port_${portStatus}.png`);
      switch (port.duplex) {
        case 'Full':
          portColor = 'background-color: greenyellow;';
          break;
        case 'Half':
          portColor = 'background-color: darkorange;';
          break;
        default:
          portColor = 'background-color: whitesmoke;';
      }
      const txBytes = netdevs[portName].stats.tx_bytes;
      const rxBytes = netdevs[portName].stats.rx_bytes;
      portDIV = E('div', { style: boxStyle }, [
        E('div', { style: boxHeadStyle + portColor }, portName),
        E('div', { style: boxbodyStyle }, [
          E('img', { src: icon, style: iconStyle }),
          E('div', { style: speedStyle }, port.speed),
          E('div', { style: trafficStyle }, [
            '\u25b2\u202f%1024.1mB'.format(txBytes),
            E('br'),
            '\u25bc\u202f%1024.1mB'.format(rxBytes)
          ])
        ])
      ]);
      if (portName == 'wan') {
        wanPortDIV = portDIV;
      } else {
        ports.push(portDIV);
      }
    }
    ports.unshift(wanPortDIV);

    const gridStyle =
      'display:grid;' +
      'grid-gap: 5px 5px;' +
      'grid-template-columns:repeat(auto-fit, minmax(70px, 1fr));' +
      'margin-bottom:1em';
    return E('div', { style: gridStyle }, ports);
  }
});
