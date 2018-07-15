module.exports = function (link, ctx) {
  //ctx.lineStyle(1, 0xFFFFFF, 1);
  ctx.lineStyle(link.width, 0xFFFFFF, 1);
  ctx.moveTo(link.from.x, link.from.y);
  ctx.lineTo(link.to.x, link.to.y);
}
