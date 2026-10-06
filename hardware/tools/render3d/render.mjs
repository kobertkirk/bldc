import { chromium } from 'playwright-core';
const [,, file, outPrefix, ...views] = process.argv;   // view: name:az:el:dist[:tx:tz]
const browser = await chromium.launch({ executablePath: process.env.CHROMIUM || '/opt/pw-browsers/chromium', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
page.on('console', m => console.log('page:', m.text()));
await page.goto(`http://127.0.0.1:8765/index.html?f=${file}`);
await page.waitForFunction(() => window.ready || window.err, null, { timeout: 600000 });
const err = await page.evaluate(() => window.err); if (err) { console.log('ERR', err); process.exit(1); }
console.log('fit', JSON.stringify(await page.evaluate(() => window.fit)));
for (const v of views) {
  const [name, ...n] = v.split(':'); const a = n.map(Number);
  await page.evaluate(a => window.view(...a), a);
  await page.screenshot({ path: `${outPrefix}-${name}.png` });
  console.log('wrote', `${outPrefix}-${name}.png`);
}
if (process.env.GLB) {
  const b64 = await page.evaluate(() => window.exportGLB());
  (await import('fs')).writeFileSync(process.env.GLB, Buffer.from(b64, 'base64'));
  console.log('wrote', process.env.GLB);
}
await browser.close();
