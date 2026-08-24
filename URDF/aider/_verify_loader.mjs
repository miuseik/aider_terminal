import puppeteer from 'puppeteer-core';

const browser = await puppeteer.launch({
  executablePath: '/snap/bin/chromium',
  headless: 'new',
  args: ['--no-sandbox', '--disable-setuid-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage();

const errors = [];
const failed = [];
page.on('console', (msg) => {
  if (msg.type() === 'error') errors.push('CONSOLE ERROR: ' + msg.text());
});
page.on('pageerror', (err) => errors.push('PAGE ERROR: ' + err.message));
page.on('requestfailed', (req) =>
  failed.push('REQ FAILED: ' + req.url() + ' -> ' + (req.failure()?.errorText || '?'))
);
page.on('response', (res) => {
  if (res.status() >= 400) failed.push('HTTP ' + res.status() + ': ' + res.url());
});

await page.goto('http://localhost:8099/viewer.html', { waitUntil: 'networkidle2', timeout: 60000 });

let status = 'TIMEOUT';
try {
  await page.waitForFunction(
    () => {
      const s = document.getElementById('status');
      return s && (s.textContent.includes('加载完成') || s.textContent.includes('Error') || s.textContent.includes('error'));
    },
    { timeout: 30000 }
  );
  status = await page.$eval('#status', (e) => e.textContent);
} catch (e) {
  status = 'TIMEOUT; current: ' + (await page.$eval('#status', (e) => e.textContent).catch(() => 'no status el'));
}

const jointCount = await page.evaluate(() => document.querySelectorAll('#sliders input[type=range]').length);
await page.screenshot({ path: '/tmp/viewer_shot.png' });

console.log('=== STATUS ===', status);
console.log('=== SLIDER COUNT ===', jointCount);
console.log('=== ERRORS ===', errors.length ? '\n' + errors.join('\n') : '(none)');
console.log('=== FAILED ===', failed.length ? '\n' + failed.join('\n') : '(none)');

await browser.close();
