module.exports = function (link) {
  return {
    width: 67 - Math.log(link.toId)
  }
};
