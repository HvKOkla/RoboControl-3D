const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const { execFileSync } = require('node:child_process');

test('exports the active DH chain and joint limits as URDF', () => {
  const source = fs.readFileSync('src/script.js', 'utf8');
  const start = source.indexOf('  exportURDF(dhParams, jointLimits) {');
  assert.notEqual(start, -1, 'URDF exporter exists');
  const end = source.indexOf('\n  downloadFile(dataStr, filename) {', start);
  assert.notEqual(end, -1, 'URDF exporter ends before downloadFile');
  const method = source.slice(start, end).replace(/^  /gm, '');
  const runner = vm.runInNewContext(`({${method}})`);
  let downloaded;
  runner.downloadFile = (data, filename) => { downloaded = { data, filename }; };
  runner.exportURDF(
    [{ a: 0, alpha: Math.PI / 2, d: 15, thetaOffset: 0 }, { a: 30, alpha: 0, d: 0, thetaOffset: 0 }],
    [{ min: -150, max: 150 }, { min: -60, max: 90 }]
  );

  assert.equal(downloaded.filename, 'robot.urdf');
  const xml = decodeURIComponent(downloaded.data.split(',')[1]);
  assert.match(xml, /<parent link="base_link"\/>/);
  assert.match(xml, /<child link="link_2"\/>/);
  assert.match(xml, /<joint name="joint_1" type="revolute">[\s\S]*?<origin xyz="0 0 0\.150000" rpy="0 0 0\.000000"\/>[\s\S]*?<axis xyz="0 0 1"\/>/);
  assert.match(xml, /<link name="base_link"\/>[\s\S]*?<link name="link_1"\/>[\s\S]*?<link name="link_2"\/>/);
  assert.match(xml, /<joint name="link_transform_1" type="fixed">[\s\S]*?<origin xyz="0\.000000 0 0" rpy="1\.570796 0 0"\/>/);
  assert.match(xml, /<joint name="link_transform_2" type="fixed">[\s\S]*?<origin xyz="0\.300000 0 0" rpy="0\.000000 0 0"\/>/);
  assert.match(xml, /lower="-2\.617994" upper="2\.617994"/);
  assert.match(xml, /lower="-1\.047198" upper="1\.570796"/);
  execFileSync('python', ['-c', 'import sys,xml.etree.ElementTree as ET; root=ET.fromstring(sys.stdin.read()); links={x.attrib["name"] for x in root.findall("link")}; joints=root.findall("joint"); names=[j.attrib["name"] for j in joints]; assert len(links)==len(root.findall("link")); assert len(names)==len(set(names)); assert all(j.find("parent").attrib["link"] in links and j.find("child").attrib["link"] in links for j in joints); assert all(j.find("axis") is not None and j.find("limit") is not None for j in joints if j.attrib["type"]=="revolute")'], { input: xml, encoding: 'utf8' });
});
