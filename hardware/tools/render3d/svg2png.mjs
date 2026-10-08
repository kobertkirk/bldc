// Rasterise an SVG to PNG in headless Chromium: node svg2png.mjs in.svg out.png [width_px]
import { chromium } from './node_modules/playwright-core/index.mjs';
const [,, svg, png, width = '1800'] = process.argv;
const b = await chromium.launch({ executablePath: process.env.CHROMIUM || '/opt/pw-browsers/chromium' });
const p = await b.newPage({ viewport: { width: Number(width) + 40, height: 1200 } });
await p.goto('file://' + svg);
await p.evaluate(w => { const s = document.querySelector('svg'); s.style.width = w + 'px'; s.style.height = 'auto';
  s.style.background = 'white'; }, width);
await (await p.$('svg')).screenshot({ path: png });
await b.close();
