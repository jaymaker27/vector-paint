async function send(cmd) {
  const response = await fetch(`/${cmd}`);
  console.log("Response:", await response.text());
}

// Canvas setup
const canvas = document.getElementById("paintCanvas");
const ctx = canvas.getContext("2d");
canvas.width = window.innerWidth - 220;
canvas.height = window.innerHeight;

const gun = document.getElementById("gun");

function shootSplat() {
  const x = Math.random() * (canvas.width - 50) + 50;
  const y = Math.random() * (canvas.height - 100) + 50;

  const gunX = gun.offsetLeft + 20;
  const gunY = gun.offsetTop + 60;
  const angle = Math.atan2(y - gunY, x - gunX) * 180 / Math.PI;
  gun.style.transform = `rotate(${angle}deg)`;

  const radius = Math.random() * 25 + 15;
  const spikes = 6 + Math.floor(Math.random() * 4);
  const color = `hsl(${Math.random()*360}, 100%, 50%)`;

  ctx.fillStyle = color;
  ctx.beginPath();
  for(let i=0;i<spikes;i++){
    const theta = (i/spikes)*Math.PI*2;
    const r = radius * (0.7 + Math.random()*0.6);
    const sx = x + r * Math.cos(theta);
    const sy = y + r * Math.sin(theta);
    if(i===0) ctx.moveTo(sx,sy);
    else ctx.lineTo(sx,sy);
  }
  ctx.closePath();
  ctx.fill();
}

setInterval(() => { shootSplat(); }, Math.random()*500 + 700);

window.addEventListener('resize', () => {
  canvas.width = window.innerWidth - 220;
  canvas.height = window.innerHeight;
});

