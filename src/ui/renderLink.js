module.exports = function (link, ctx) {
  ctx.lineStyle(1, 0x7B5C95, 1);
  //ctx.lineStyle(link.width, 0x7B5C95, 2);
  ctx.moveTo(link.from.x, link.from.y);
  ctx.lineTo(link.to.x, link.to.y);
}
