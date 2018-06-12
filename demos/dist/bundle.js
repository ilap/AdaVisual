(function(f){if(typeof exports==="object"&&typeof module!=="undefined"){module.exports=f()}else if(typeof define==="function"&&define.amd){define([],f)}else{var g;if(typeof window!=="undefined"){g=window}else if(typeof global!=="undefined"){g=global}else if(typeof self!=="undefined"){g=self}else{g=this}g.ngraph = f()}})(function(){var define,module,exports;return (function(){function r(e,n,t){function o(i,f){if(!n[i]){if(!e[i]){var c="function"==typeof require&&require;if(!f&&c)return c(i,!0);if(u)return u(i,!0);var a=new Error("Cannot find module '"+i+"'");throw a.code="MODULE_NOT_FOUND",a}var p=n[i]={exports:{}};e[i][0].call(p.exports,function(r){var n=e[i][1][r];return o(n||r)},p,p.exports,r,e,n,t)}return n[i].exports}for(var u="function"==typeof require&&require,i=0;i<t.length;i++)o(t[i]);return o}return r})()({1:[function(require,module,exports){
module.exports = function(subject) {
  validateSubject(subject);

  var eventsStorage = createEventsStorage(subject);
  subject.on = eventsStorage.on;
  subject.off = eventsStorage.off;
  subject.fire = eventsStorage.fire;
  return subject;
};

function createEventsStorage(subject) {
  // Store all event listeners to this hash. Key is event name, value is array
  // of callback records.
  //
  // A callback record consists of callback function and its optional context:
  // { 'eventName' => [{callback: function, ctx: object}] }
  var registeredEvents = Object.create(null);

  return {
    on: function (eventName, callback, ctx) {
      if (typeof callback !== 'function') {
        throw new Error('callback is expected to be a function');
      }
      var handlers = registeredEvents[eventName];
      if (!handlers) {
        handlers = registeredEvents[eventName] = [];
      }
      handlers.push({callback: callback, ctx: ctx});

      return subject;
    },

    off: function (eventName, callback) {
      var wantToRemoveAll = (typeof eventName === 'undefined');
      if (wantToRemoveAll) {
        // Killing old events storage should be enough in this case:
        registeredEvents = Object.create(null);
        return subject;
      }

      if (registeredEvents[eventName]) {
        var deleteAllCallbacksForEvent = (typeof callback !== 'function');
        if (deleteAllCallbacksForEvent) {
          delete registeredEvents[eventName];
        } else {
          var callbacks = registeredEvents[eventName];
          for (var i = 0; i < callbacks.length; ++i) {
            if (callbacks[i].callback === callback) {
              callbacks.splice(i, 1);
            }
          }
        }
      }

      return subject;
    },

    fire: function (eventName) {
      var callbacks = registeredEvents[eventName];
      if (!callbacks) {
        return subject;
      }

      var fireArguments;
      if (arguments.length > 1) {
        fireArguments = Array.prototype.splice.call(arguments, 1);
      }
      for(var i = 0; i < callbacks.length; ++i) {
        var callbackInfo = callbacks[i];
        callbackInfo.callback.apply(callbackInfo.ctx, fireArguments);
      }

      return subject;
    }
  };
}

function validateSubject(subject) {
  if (!subject) {
    throw new Error('Eventify cannot use falsy object as events subject');
  }
  var reservedWords = ['on', 'fire', 'off'];
  for (var i = 0; i < reservedWords.length; ++i) {
    if (subject.hasOwnProperty(reservedWords[i])) {
      throw new Error("Subject cannot be eventified, since it already has property '" + reservedWords[i] + "'");
    }
  }
}

},{}],2:[function(require,module,exports){
module.exports = exposeProperties;

/**
 * Augments `target` object with getter/setter functions, which modify settings
 *
 * @example
 *  var target = {};
 *  exposeProperties({ age: 42}, target);
 *  target.age(); // returns 42
 *  target.age(24); // make age 24;
 *
 *  var filteredTarget = {};
 *  exposeProperties({ age: 42, name: 'John'}, filteredTarget, ['name']);
 *  filteredTarget.name(); // returns 'John'
 *  filteredTarget.age === undefined; // true
 */
function exposeProperties(settings, target, filter) {
  var needsFilter = Object.prototype.toString.call(filter) === '[object Array]';
  if (needsFilter) {
    for (var i = 0; i < filter.length; ++i) {
      augment(settings, target, filter[i]);
    }
  } else {
    for (var key in settings) {
      augment(settings, target, key);
    }
  }
}

function augment(source, target, key) {
  if (source.hasOwnProperty(key)) {
    if (typeof target[key] === 'function') {
      // this accessor is already defined. Ignore it
      return;
    }
    target[key] = function (value) {
      if (value !== undefined) {
        source[key] = value;
        return target;
      }
      return source[key];
    }
  }
}

},{}],3:[function(require,module,exports){
module.exports = createLayout;
module.exports.simulator = require('ngraph.physics.simulator');

var eventify = require('ngraph.events');

/**
 * Creates force based layout for a given graph.
 *
 * @param {ngraph.graph} graph which needs to be laid out
 * @param {object} physicsSettings if you need custom settings
 * for physics simulator you can pass your own settings here. If it's not passed
 * a default one will be created.
 */
function createLayout(graph, physicsSettings) {
  if (!graph) {
    throw new Error('Graph structure cannot be undefined');
  }

  var createSimulator = require('ngraph.physics.simulator');
  var physicsSimulator = createSimulator(physicsSettings);

  var nodeBodies = Object.create(null);
  var springs = {};
  var bodiesCount = 0;

  var springTransform = physicsSimulator.settings.springTransform || noop;

  // Initialize physics with what we have in the graph:
  initPhysics();
  listenToEvents();

  var wasStable = false;

  var api = {
    /**
     * Performs one step of iterative layout algorithm
     *
     * @returns {boolean} true if the system should be considered stable; Flase otherwise.
     * The system is stable if no further call to `step()` can improve the layout.
     */
    step: function() {
      if (bodiesCount === 0) return true; // TODO: This will never fire 'stable'

      var lastMove = physicsSimulator.step();

      // Save the movement in case if someone wants to query it in the step
      // callback.
      api.lastMove = lastMove;

      // Allow listeners to perform low-level actions after nodes are updated.
      api.fire('step');

      var ratio = lastMove/bodiesCount;
      var isStableNow = ratio <= 0.01; // TODO: The number is somewhat arbitrary...

      if (wasStable !== isStableNow) {
        wasStable = isStableNow;
        onStableChanged(isStableNow);
      }

      return isStableNow;
    },

    /**
     * For a given `nodeId` returns position
     */
    getNodePosition: function (nodeId) {
      return getInitializedBody(nodeId).pos;
    },

    /**
     * Sets position of a node to a given coordinates
     * @param {string} nodeId node identifier
     * @param {number} x position of a node
     * @param {number} y position of a node
     * @param {number=} z position of node (only if applicable to body)
     */
    setNodePosition: function (nodeId) {
      var body = getInitializedBody(nodeId);
      body.setPosition.apply(body, Array.prototype.slice.call(arguments, 1));
    },

    /**
     * @returns {Object} Link position by link id
     * @returns {Object.from} {x, y} coordinates of link start
     * @returns {Object.to} {x, y} coordinates of link end
     */
    getLinkPosition: function (linkId) {
      var spring = springs[linkId];
      if (spring) {
        return {
          from: spring.from.pos,
          to: spring.to.pos
        };
      }
    },

    /**
     * @returns {Object} area required to fit in the graph. Object contains
     * `x1`, `y1` - top left coordinates
     * `x2`, `y2` - bottom right coordinates
     */
    getGraphRect: function () {
      return physicsSimulator.getBBox();
    },

    /**
     * Iterates over each body in the layout simulator and performs a callback(body, nodeId)
     */
    forEachBody: forEachBody,

    /*
     * Requests layout algorithm to pin/unpin node to its current position
     * Pinned nodes should not be affected by layout algorithm and always
     * remain at their position
     */
    pinNode: function (node, isPinned) {
      var body = getInitializedBody(node.id);
       body.isPinned = !!isPinned;
    },

    /**
     * Checks whether given graph's node is currently pinned
     */
    isNodePinned: function (node) {
      return getInitializedBody(node.id).isPinned;
    },

    /**
     * Request to release all resources
     */
    dispose: function() {
      graph.off('changed', onGraphChanged);
      api.fire('disposed');
    },

    /**
     * Gets physical body for a given node id. If node is not found undefined
     * value is returned.
     */
    getBody: getBody,

    /**
     * Gets spring for a given edge.
     *
     * @param {string} linkId link identifer. If two arguments are passed then
     * this argument is treated as formNodeId
     * @param {string=} toId when defined this parameter denotes head of the link
     * and first argument is trated as tail of the link (fromId)
     */
    getSpring: getSpring,

    /**
     * [Read only] Gets current physics simulator
     */
    simulator: physicsSimulator,

    /**
     * Gets the graph that was used for layout
     */
    graph: graph,

    /**
     * Gets amount of movement performed during last step opeartion
     */
    lastMove: 0
  };

  eventify(api);

  return api;

  function forEachBody(cb) {
    Object.keys(nodeBodies).forEach(function(bodyId) {
      cb(nodeBodies[bodyId], bodyId);
    });
  }

  function getSpring(fromId, toId) {
    var linkId;
    if (toId === undefined) {
      if (typeof fromId !== 'object') {
        // assume fromId as a linkId:
        linkId = fromId;
      } else {
        // assume fromId to be a link object:
        linkId = fromId.id;
      }
    } else {
      // toId is defined, should grab link:
      var link = graph.hasLink(fromId, toId);
      if (!link) return;
      linkId = link.id;
    }

    return springs[linkId];
  }

  function getBody(nodeId) {
    return nodeBodies[nodeId];
  }

  function listenToEvents() {
    graph.on('changed', onGraphChanged);
  }

  function onStableChanged(isStable) {
    api.fire('stable', isStable);
  }

  function onGraphChanged(changes) {
    for (var i = 0; i < changes.length; ++i) {
      var change = changes[i];
      if (change.changeType === 'add') {
        if (change.node) {
          initBody(change.node.id);
        }
        if (change.link) {
          initLink(change.link);
        }
      } else if (change.changeType === 'remove') {
        if (change.node) {
          releaseNode(change.node);
        }
        if (change.link) {
          releaseLink(change.link);
        }
      }
    }
    bodiesCount = graph.getNodesCount();
  }

  function initPhysics() {
    bodiesCount = 0;

    graph.forEachNode(function (node) {
      initBody(node.id);
      bodiesCount += 1;
    });

    graph.forEachLink(initLink);
  }

  function initBody(nodeId) {
    var body = nodeBodies[nodeId];
    if (!body) {
      var node = graph.getNode(nodeId);
      if (!node) {
        throw new Error('initBody() was called with unknown node id');
      }

      var pos = node.position;
      if (!pos) {
        var neighbors = getNeighborBodies(node);
        pos = physicsSimulator.getBestNewBodyPosition(neighbors);
      }

      body = physicsSimulator.addBodyAt(pos);
      body.id = nodeId;

      nodeBodies[nodeId] = body;
      updateBodyMass(nodeId);

      if (isNodeOriginallyPinned(node)) {
        body.isPinned = true;
      }
    }
  }

  function releaseNode(node) {
    var nodeId = node.id;
    var body = nodeBodies[nodeId];
    if (body) {
      nodeBodies[nodeId] = null;
      delete nodeBodies[nodeId];

      physicsSimulator.removeBody(body);
    }
  }

  function initLink(link) {
    updateBodyMass(link.fromId);
    updateBodyMass(link.toId);

    var fromBody = nodeBodies[link.fromId],
        toBody  = nodeBodies[link.toId],
        spring = physicsSimulator.addSpring(fromBody, toBody, link.length);

    springTransform(link, spring);

    springs[link.id] = spring;
  }

  function releaseLink(link) {
    var spring = springs[link.id];
    if (spring) {
      var from = graph.getNode(link.fromId),
          to = graph.getNode(link.toId);

      if (from) updateBodyMass(from.id);
      if (to) updateBodyMass(to.id);

      delete springs[link.id];

      physicsSimulator.removeSpring(spring);
    }
  }

  function getNeighborBodies(node) {
    // TODO: Could probably be done better on memory
    var neighbors = [];
    if (!node.links) {
      return neighbors;
    }
    var maxNeighbors = Math.min(node.links.length, 2);
    for (var i = 0; i < maxNeighbors; ++i) {
      var link = node.links[i];
      var otherBody = link.fromId !== node.id ? nodeBodies[link.fromId] : nodeBodies[link.toId];
      if (otherBody && otherBody.pos) {
        neighbors.push(otherBody);
      }
    }

    return neighbors;
  }

  function updateBodyMass(nodeId) {
    var body = nodeBodies[nodeId];
    body.mass = nodeMass(nodeId);
  }

  /**
   * Checks whether graph node has in its settings pinned attribute,
   * which means layout algorithm cannot move it. Node can be preconfigured
   * as pinned, if it has "isPinned" attribute, or when node.data has it.
   *
   * @param {Object} node a graph node to check
   * @return {Boolean} true if node should be treated as pinned; false otherwise.
   */
  function isNodeOriginallyPinned(node) {
    return (node && (node.isPinned || (node.data && node.data.isPinned)));
  }

  function getInitializedBody(nodeId) {
    var body = nodeBodies[nodeId];
    if (!body) {
      initBody(nodeId);
      body = nodeBodies[nodeId];
    }
    return body;
  }

  /**
   * Calculates mass of a body, which corresponds to node with given id.
   *
   * @param {String|Number} nodeId identifier of a node, for which body mass needs to be calculated
   * @returns {Number} recommended mass of the body;
   */
  function nodeMass(nodeId) {
    var links = graph.getLinks(nodeId);
    if (!links) return 1;
    return 1 + links.length / 3.0;
  }
}

function noop() { }

},{"ngraph.events":1,"ngraph.physics.simulator":4}],4:[function(require,module,exports){
/**
 * Manages a simulation of physical forces acting on bodies and springs.
 */
module.exports = physicsSimulator;

function physicsSimulator(settings) {
  var Spring = require('./lib/spring');
  var expose = require('ngraph.expose');
  var merge = require('ngraph.merge');
  var eventify = require('ngraph.events');

  settings = merge(settings, {
      /**
       * Ideal length for links (springs in physical model).
       */
      springLength: 30,

      /**
       * Hook's law coefficient. 1 - solid spring.
       */
      springCoeff: 0.0008,

      /**
       * Coulomb's law coefficient. It's used to repel nodes thus should be negative
       * if you make it positive nodes start attract each other :).
       */
      gravity: -1.2,

      /**
       * Theta coefficient from Barnes Hut simulation. Ranged between (0, 1).
       * The closer it's to 1 the more nodes algorithm will have to go through.
       * Setting it to one makes Barnes Hut simulation no different from
       * brute-force forces calculation (each node is considered).
       */
      theta: 0.8,

      /**
       * Drag force coefficient. Used to slow down system, thus should be less than 1.
       * The closer it is to 0 the less tight system will be.
       */
      dragCoeff: 0.02,

      /**
       * Default time step (dt) for forces integration
       */
      timeStep : 20,
  });

  // We allow clients to override basic factory methods:
  var createQuadTree = settings.createQuadTree || require('ngraph.quadtreebh');
  var createBounds = settings.createBounds || require('./lib/bounds');
  var createDragForce = settings.createDragForce || require('./lib/dragForce');
  var createSpringForce = settings.createSpringForce || require('./lib/springForce');
  var integrate = settings.integrator || require('./lib/eulerIntegrator');
  var createBody = settings.createBody || require('./lib/createBody');

  var bodies = [], // Bodies in this simulation.
      springs = [], // Springs in this simulation.
      quadTree =  createQuadTree(settings),
      bounds = createBounds(bodies, settings),
      springForce = createSpringForce(settings),
      dragForce = createDragForce(settings);

  var totalMovement = 0; // how much movement we made on last step

  var publicApi = {
    /**
     * Array of bodies, registered with current simulator
     *
     * Note: To add new body, use addBody() method. This property is only
     * exposed for testing/performance purposes.
     */
    bodies: bodies,

    quadTree: quadTree,

    /**
     * Array of springs, registered with current simulator
     *
     * Note: To add new spring, use addSpring() method. This property is only
     * exposed for testing/performance purposes.
     */
    springs: springs,

    /**
     * Returns settings with which current simulator was initialized
     */
    settings: settings,

    /**
     * Performs one step of force simulation.
     *
     * @returns {boolean} true if system is considered stable; False otherwise.
     */
    step: function () {
      accumulateForces();

      var movement = integrate(bodies, settings.timeStep);
      bounds.update();

      return movement;
    },

    /**
     * Adds body to the system
     *
     * @param {ngraph.physics.primitives.Body} body physical body
     *
     * @returns {ngraph.physics.primitives.Body} added body
     */
    addBody: function (body) {
      if (!body) {
        throw new Error('Body is required');
      }
      bodies.push(body);

      return body;
    },

    /**
     * Adds body to the system at given position
     *
     * @param {Object} pos position of a body
     *
     * @returns {ngraph.physics.primitives.Body} added body
     */
    addBodyAt: function (pos) {
      if (!pos) {
        throw new Error('Body position is required');
      }
      var body = createBody(pos);
      bodies.push(body);

      return body;
    },

    /**
     * Removes body from the system
     *
     * @param {ngraph.physics.primitives.Body} body to remove
     *
     * @returns {Boolean} true if body found and removed. falsy otherwise;
     */
    removeBody: function (body) {
      if (!body) { return; }

      var idx = bodies.indexOf(body);
      if (idx < 0) { return; }

      bodies.splice(idx, 1);
      if (bodies.length === 0) {
        bounds.reset();
      }
      return true;
    },

    /**
     * Adds a spring to this simulation.
     *
     * @returns {Object} - a handle for a spring. If you want to later remove
     * spring pass it to removeSpring() method.
     */
    addSpring: function (body1, body2, springLength, springWeight, springCoefficient) {
      if (!body1 || !body2) {
        throw new Error('Cannot add null spring to force simulator');
      }

      if (typeof springLength !== 'number') {
        springLength = -1; // assume global configuration
      }

      var spring = new Spring(body1, body2, springLength, springCoefficient >= 0 ? springCoefficient : -1, springWeight);
      springs.push(spring);

      // TODO: could mark simulator as dirty.
      return spring;
    },

    /**
     * Returns amount of movement performed on last step() call
     */
    getTotalMovement: function () {
      return totalMovement;
    },

    /**
     * Removes spring from the system
     *
     * @param {Object} spring to remove. Spring is an object returned by addSpring
     *
     * @returns {Boolean} true if spring found and removed. falsy otherwise;
     */
    removeSpring: function (spring) {
      if (!spring) { return; }
      var idx = springs.indexOf(spring);
      if (idx > -1) {
        springs.splice(idx, 1);
        return true;
      }
    },

    getBestNewBodyPosition: function (neighbors) {
      return bounds.getBestNewPosition(neighbors);
    },

    /**
     * Returns bounding box which covers all bodies
     */
    getBBox: function () {
      return bounds.box;
    },

    gravity: function (value) {
      if (value !== undefined) {
        settings.gravity = value;
        quadTree.options({gravity: value});
        return this;
      } else {
        return settings.gravity;
      }
    },

    theta: function (value) {
      if (value !== undefined) {
        settings.theta = value;
        quadTree.options({theta: value});
        return this;
      } else {
        return settings.theta;
      }
    }
  };

  // allow settings modification via public API:
  expose(settings, publicApi);

  eventify(publicApi);

  return publicApi;

  function accumulateForces() {
    // Accumulate forces acting on bodies.
    var body,
        i = bodies.length;

    if (i) {
      // only add bodies if there the array is not empty:
      quadTree.insertBodies(bodies); // performance: O(n * log n)
      while (i--) {
        body = bodies[i];
        // If body is pinned there is no point updating its forces - it should
        // never move:
        if (!body.isPinned) {
          body.force.reset();

          quadTree.updateBodyForce(body);
          dragForce.update(body);
        }
      }
    }

    i = springs.length;
    while(i--) {
      springForce.update(springs[i]);
    }
  }
};

},{"./lib/bounds":5,"./lib/createBody":6,"./lib/dragForce":7,"./lib/eulerIntegrator":8,"./lib/spring":9,"./lib/springForce":10,"ngraph.events":1,"ngraph.expose":2,"ngraph.merge":13,"ngraph.quadtreebh":27}],5:[function(require,module,exports){
module.exports = function (bodies, settings) {
  var random = require('ngraph.random').random(42);
  var boundingBox =  { x1: 0, y1: 0, x2: 0, y2: 0 };

  return {
    box: boundingBox,

    update: updateBoundingBox,

    reset : function () {
      boundingBox.x1 = boundingBox.y1 = 0;
      boundingBox.x2 = boundingBox.y2 = 0;
    },

    getBestNewPosition: function (neighbors) {
      var graphRect = boundingBox;

      var baseX = 0, baseY = 0;

      if (neighbors.length) {
        for (var i = 0; i < neighbors.length; ++i) {
          baseX += neighbors[i].pos.x;
          baseY += neighbors[i].pos.y;
        }

        baseX /= neighbors.length;
        baseY /= neighbors.length;
      } else {
        baseX = (graphRect.x1 + graphRect.x2) / 2;
        baseY = (graphRect.y1 + graphRect.y2) / 2;
      }

      var springLength = settings.springLength;
      return {
        x: baseX + random.next(springLength) - springLength / 2,
        y: baseY + random.next(springLength) - springLength / 2
      };
    }
  };

  function updateBoundingBox() {
    var i = bodies.length;
    if (i === 0) { return; } // don't have to wory here.

    var x1 = Number.MAX_VALUE,
        y1 = Number.MAX_VALUE,
        x2 = Number.MIN_VALUE,
        y2 = Number.MIN_VALUE;

    while(i--) {
      // this is O(n), could it be done faster with quadtree?
      // how about pinned nodes?
      var body = bodies[i];
      if (body.isPinned) {
        body.pos.x = body.prevPos.x;
        body.pos.y = body.prevPos.y;
      } else {
        body.prevPos.x = body.pos.x;
        body.prevPos.y = body.pos.y;
      }
      if (body.pos.x < x1) {
        x1 = body.pos.x;
      }
      if (body.pos.x > x2) {
        x2 = body.pos.x;
      }
      if (body.pos.y < y1) {
        y1 = body.pos.y;
      }
      if (body.pos.y > y2) {
        y2 = body.pos.y;
      }
    }

    boundingBox.x1 = x1;
    boundingBox.x2 = x2;
    boundingBox.y1 = y1;
    boundingBox.y2 = y2;
  }
}

},{"ngraph.random":11}],6:[function(require,module,exports){
var physics = require('ngraph.physics.primitives');

module.exports = function(pos) {
  return new physics.Body(pos);
}

},{"ngraph.physics.primitives":14}],7:[function(require,module,exports){
/**
 * Represents drag force, which reduces force value on each step by given
 * coefficient.
 *
 * @param {Object} options for the drag force
 * @param {Number=} options.dragCoeff drag force coefficient. 0.1 by default
 */
module.exports = function (options) {
  var merge = require('ngraph.merge'),
      expose = require('ngraph.expose');

  options = merge(options, {
    dragCoeff: 0.02
  });

  var api = {
    update : function (body) {
      body.force.x -= options.dragCoeff * body.velocity.x;
      body.force.y -= options.dragCoeff * body.velocity.y;
    }
  };

  // let easy access to dragCoeff:
  expose(options, api, ['dragCoeff']);

  return api;
};

},{"ngraph.expose":2,"ngraph.merge":13}],8:[function(require,module,exports){
/**
 * Performs forces integration, using given timestep. Uses Euler method to solve
 * differential equation (http://en.wikipedia.org/wiki/Euler_method ).
 *
 * @returns {Number} squared distance of total position updates.
 */

module.exports = integrate;

function integrate(bodies, timeStep) {
  var dx = 0, tx = 0,
      dy = 0, ty = 0,
      i,
      max = bodies.length;

  if (max === 0) {
    return 0;
  }

  for (i = 0; i < max; ++i) {
    var body = bodies[i],
        coeff = timeStep / body.mass;

    body.velocity.x += coeff * body.force.x;
    body.velocity.y += coeff * body.force.y;
    var vx = body.velocity.x,
        vy = body.velocity.y,
        v = Math.sqrt(vx * vx + vy * vy);

    if (v > 1) {
      body.velocity.x = vx / v;
      body.velocity.y = vy / v;
    }

    dx = timeStep * body.velocity.x;
    dy = timeStep * body.velocity.y;

    body.pos.x += dx;
    body.pos.y += dy;

    tx += Math.abs(dx); ty += Math.abs(dy);
  }

  return (tx * tx + ty * ty)/max;
}

},{}],9:[function(require,module,exports){
module.exports = Spring;

/**
 * Represents a physical spring. Spring connects two bodies, has rest length
 * stiffness coefficient and optional weight
 */
function Spring(fromBody, toBody, length, coeff, weight) {
    this.from = fromBody;
    this.to = toBody;
    this.length = length;
    this.coeff = coeff;

    this.weight = typeof weight === 'number' ? weight : 1;
};

},{}],10:[function(require,module,exports){
/**
 * Represents spring force, which updates forces acting on two bodies, conntected
 * by a spring.
 *
 * @param {Object} options for the spring force
 * @param {Number=} options.springCoeff spring force coefficient.
 * @param {Number=} options.springLength desired length of a spring at rest.
 */
module.exports = function (options) {
  var merge = require('ngraph.merge');
  var random = require('ngraph.random').random(42);
  var expose = require('ngraph.expose');

  options = merge(options, {
    springCoeff: 0.0002,
    springLength: 80
  });

  var api = {
    /**
     * Upsates forces acting on a spring
     */
    update : function (spring) {
      var body1 = spring.from,
          body2 = spring.to,
          length = spring.length < 0 ? options.springLength : spring.length,
          dx = body2.pos.x - body1.pos.x,
          dy = body2.pos.y - body1.pos.y,
          r = Math.sqrt(dx * dx + dy * dy);

      if (r === 0) {
          dx = (random.nextDouble() - 0.5) / 50;
          dy = (random.nextDouble() - 0.5) / 50;
          r = Math.sqrt(dx * dx + dy * dy);
      }

      var d = r - length;
      var coeff = ((!spring.coeff || spring.coeff < 0) ? options.springCoeff : spring.coeff) * d / r * spring.weight;

      body1.force.x += coeff * dx;
      body1.force.y += coeff * dy;

      body2.force.x -= coeff * dx;
      body2.force.y -= coeff * dy;
    }
  };

  expose(options, api, ['springCoeff', 'springLength']);
  return api;
}

},{"ngraph.expose":2,"ngraph.merge":13,"ngraph.random":11}],11:[function(require,module,exports){
module.exports = {
  random: random,
  randomIterator: randomIterator
};

/**
 * Creates seeded PRNG with two methods:
 *   next() and nextDouble()
 */
function random(inputSeed) {
  var seed = typeof inputSeed === 'number' ? inputSeed : (+ new Date());
  var randomFunc = function() {
      // Robert Jenkins' 32 bit integer hash function.
      seed = ((seed + 0x7ed55d16) + (seed << 12))  & 0xffffffff;
      seed = ((seed ^ 0xc761c23c) ^ (seed >>> 19)) & 0xffffffff;
      seed = ((seed + 0x165667b1) + (seed << 5))   & 0xffffffff;
      seed = ((seed + 0xd3a2646c) ^ (seed << 9))   & 0xffffffff;
      seed = ((seed + 0xfd7046c5) + (seed << 3))   & 0xffffffff;
      seed = ((seed ^ 0xb55a4f09) ^ (seed >>> 16)) & 0xffffffff;
      return (seed & 0xfffffff) / 0x10000000;
  };

  return {
      /**
       * Generates random integer number in the range from 0 (inclusive) to maxValue (exclusive)
       *
       * @param maxValue Number REQUIRED. Ommitting this number will result in NaN values from PRNG.
       */
      next : function (maxValue) {
          return Math.floor(randomFunc() * maxValue);
      },

      /**
       * Generates random double number in the range from 0 (inclusive) to 1 (exclusive)
       * This function is the same as Math.random() (except that it could be seeded)
       */
      nextDouble : function () {
          return randomFunc();
      }
  };
}

/*
 * Creates iterator over array, which returns items of array in random order
 * Time complexity is guaranteed to be O(n);
 */
function randomIterator(array, customRandom) {
    var localRandom = customRandom || random();
    if (typeof localRandom.next !== 'function') {
      throw new Error('customRandom does not match expected API: next() function is missing');
    }

    return {
        forEach : function (callback) {
            var i, j, t;
            for (i = array.length - 1; i > 0; --i) {
                j = localRandom.next(i + 1); // i inclusive
                t = array[j];
                array[j] = array[i];
                array[i] = t;

                callback(t);
            }

            if (array.length) {
                callback(array[0]);
            }
        },

        /**
         * Shuffles array randomly, in place.
         */
        shuffle : function () {
            var i, j, t;
            for (i = array.length - 1; i > 0; --i) {
                j = localRandom.next(i + 1); // i inclusive
                t = array[j];
                array[j] = array[i];
                array[i] = t;
            }

            return array;
        }
    };
}

},{}],12:[function(require,module,exports){
/**
 * @fileOverview Contains definition of the core graph object.
 */

// TODO: need to change storage layer:
// 1. Be able to get all nodes O(1)
// 2. Be able to get number of links O(1)

/**
 * @example
 *  var graph = require('ngraph.graph')();
 *  graph.addNode(1);     // graph has one node.
 *  graph.addLink(2, 3);  // now graph contains three nodes and one link.
 *
 */
module.exports = createGraph;

var eventify = require('ngraph.events');

/**
 * Creates a new graph
 */
function createGraph(options) {
  // Graph structure is maintained as dictionary of nodes
  // and array of links. Each node has 'links' property which
  // hold all links related to that node. And general links
  // array is used to speed up all links enumeration. This is inefficient
  // in terms of memory, but simplifies coding.
  options = options || {};
  if ('uniqueLinkId' in options) {
    console.warn(
      'ngraph.graph: Starting from version 0.14 `uniqueLinkId` is deprecated.\n' +
      'Use `multigraph` option instead\n',
      '\n',
      'Note: there is also change in default behavior: From now own each graph\n'+
      'is considered to be not a multigraph by default (each edge is unique).'
    );

    options.multigraph = options.uniqueLinkId;
  }

  // Dear reader, the non-multigraphs do not guarantee that there is only
  // one link for a given pair of node. When this option is set to false
  // we can save some memory and CPU (18% faster for non-multigraph);
  if (options.multigraph === undefined) options.multigraph = false;

  var nodes = typeof Object.create === 'function' ? Object.create(null) : {},
    links = [],
    // Hash of multi-edges. Used to track ids of edges between same nodes
    multiEdges = {},
    nodesCount = 0,
    suspendEvents = 0,

    forEachNode = createNodeIterator(),
    createLink = options.multigraph ? createUniqueLink : createSingleLink,

    // Our graph API provides means to listen to graph changes. Users can subscribe
    // to be notified about changes in the graph by using `on` method. However
    // in some cases they don't use it. To avoid unnecessary memory consumption
    // we will not record graph changes until we have at least one subscriber.
    // Code below supports this optimization.
    //
    // Accumulates all changes made during graph updates.
    // Each change element contains:
    //  changeType - one of the strings: 'add', 'remove' or 'update';
    //  node - if change is related to node this property is set to changed graph's node;
    //  link - if change is related to link this property is set to changed graph's link;
    changes = [],
    recordLinkChange = noop,
    recordNodeChange = noop,
    enterModification = noop,
    exitModification = noop;

  // this is our public API:
  var graphPart = {
    /**
     * Adds node to the graph. If node with given id already exists in the graph
     * its data is extended with whatever comes in 'data' argument.
     *
     * @param nodeId the node's identifier. A string or number is preferred.
     * @param [data] additional data for the node being added. If node already
     *   exists its data object is augmented with the new one.
     *
     * @return {node} The newly added node or node with given id if it already exists.
     */
    addNode: addNode,

    /**
     * Adds a link to the graph. The function always create a new
     * link between two nodes. If one of the nodes does not exists
     * a new node is created.
     *
     * @param fromId link start node id;
     * @param toId link end node id;
     * @param [data] additional data to be set on the new link;
     *
     * @return {link} The newly created link
     */
    addLink: addLink,

    /**
     * Removes link from the graph. If link does not exist does nothing.
     *
     * @param link - object returned by addLink() or getLinks() methods.
     *
     * @returns true if link was removed; false otherwise.
     */
    removeLink: removeLink,

    /**
     * Removes node with given id from the graph. If node does not exist in the graph
     * does nothing.
     *
     * @param nodeId node's identifier passed to addNode() function.
     *
     * @returns true if node was removed; false otherwise.
     */
    removeNode: removeNode,

    /**
     * Gets node with given identifier. If node does not exist undefined value is returned.
     *
     * @param nodeId requested node identifier;
     *
     * @return {node} in with requested identifier or undefined if no such node exists.
     */
    getNode: getNode,

    /**
     * Gets number of nodes in this graph.
     *
     * @return number of nodes in the graph.
     */
    getNodesCount: function () {
      return nodesCount;
    },

    /**
     * Gets total number of links in the graph.
     */
    getLinksCount: function () {
      return links.length;
    },

    /**
     * Gets all links (inbound and outbound) from the node with given id.
     * If node with given id is not found null is returned.
     *
     * @param nodeId requested node identifier.
     *
     * @return Array of links from and to requested node if such node exists;
     *   otherwise null is returned.
     */
    getLinks: getLinks,

    /**
     * Invokes callback on each node of the graph.
     *
     * @param {Function(node)} callback Function to be invoked. The function
     *   is passed one argument: visited node.
     */
    forEachNode: forEachNode,

    /**
     * Invokes callback on every linked (adjacent) node to the given one.
     *
     * @param nodeId Identifier of the requested node.
     * @param {Function(node, link)} callback Function to be called on all linked nodes.
     *   The function is passed two parameters: adjacent node and link object itself.
     * @param oriented if true graph treated as oriented.
     */
    forEachLinkedNode: forEachLinkedNode,

    /**
     * Enumerates all links in the graph
     *
     * @param {Function(link)} callback Function to be called on all links in the graph.
     *   The function is passed one parameter: graph's link object.
     *
     * Link object contains at least the following fields:
     *  fromId - node id where link starts;
     *  toId - node id where link ends,
     *  data - additional data passed to graph.addLink() method.
     */
    forEachLink: forEachLink,

    /**
     * Suspend all notifications about graph changes until
     * endUpdate is called.
     */
    beginUpdate: enterModification,

    /**
     * Resumes all notifications about graph changes and fires
     * graph 'changed' event in case there are any pending changes.
     */
    endUpdate: exitModification,

    /**
     * Removes all nodes and links from the graph.
     */
    clear: clear,

    /**
     * Detects whether there is a link between two nodes.
     * Operation complexity is O(n) where n - number of links of a node.
     * NOTE: this function is synonim for getLink()
     *
     * @returns link if there is one. null otherwise.
     */
    hasLink: getLink,

    /**
     * Detects whether there is a node with given id
     * 
     * Operation complexity is O(1)
     * NOTE: this function is synonim for getNode()
     *
     * @returns node if there is one; Falsy value otherwise.
     */
    hasNode: getNode,

    /**
     * Gets an edge between two nodes.
     * Operation complexity is O(n) where n - number of links of a node.
     *
     * @param {string} fromId link start identifier
     * @param {string} toId link end identifier
     *
     * @returns link if there is one. null otherwise.
     */
    getLink: getLink
  };

  // this will add `on()` and `fire()` methods.
  eventify(graphPart);

  monitorSubscribers();

  return graphPart;

  function monitorSubscribers() {
    var realOn = graphPart.on;

    // replace real `on` with our temporary on, which will trigger change
    // modification monitoring:
    graphPart.on = on;

    function on() {
      // now it's time to start tracking stuff:
      graphPart.beginUpdate = enterModification = enterModificationReal;
      graphPart.endUpdate = exitModification = exitModificationReal;
      recordLinkChange = recordLinkChangeReal;
      recordNodeChange = recordNodeChangeReal;

      // this will replace current `on` method with real pub/sub from `eventify`.
      graphPart.on = realOn;
      // delegate to real `on` handler:
      return realOn.apply(graphPart, arguments);
    }
  }

  function recordLinkChangeReal(link, changeType) {
    changes.push({
      link: link,
      changeType: changeType
    });
  }

  function recordNodeChangeReal(node, changeType) {
    changes.push({
      node: node,
      changeType: changeType
    });
  }

  function addNode(nodeId, data) {
    if (nodeId === undefined) {
      throw new Error('Invalid node identifier');
    }

    enterModification();

    var node = getNode(nodeId);
    if (!node) {
      node = new Node(nodeId, data);
      nodesCount++;
      recordNodeChange(node, 'add');
    } else {
      node.data = data;
      recordNodeChange(node, 'update');
    }

    nodes[nodeId] = node;

    exitModification();
    return node;
  }

  function getNode(nodeId) {
    return nodes[nodeId];
  }

  function removeNode(nodeId) {
    var node = getNode(nodeId);
    if (!node) {
      return false;
    }

    enterModification();

    var prevLinks = node.links;
    if (prevLinks) {
      node.links = null;
      for(var i = 0; i < prevLinks.length; ++i) {
        removeLink(prevLinks[i]);
      }
    }

    delete nodes[nodeId];
    nodesCount--;

    recordNodeChange(node, 'remove');

    exitModification();

    return true;
  }


  function addLink(fromId, toId, data) {
    enterModification();

    var fromNode = getNode(fromId) || addNode(fromId);
    var toNode = getNode(toId) || addNode(toId);

    var link = createLink(fromId, toId, data);

    links.push(link);

    // TODO: this is not cool. On large graphs potentially would consume more memory.
    addLinkToNode(fromNode, link);
    if (fromId !== toId) {
      // make sure we are not duplicating links for self-loops
      addLinkToNode(toNode, link);
    }

    recordLinkChange(link, 'add');

    exitModification();

    return link;
  }

  function createSingleLink(fromId, toId, data) {
    var linkId = makeLinkId(fromId, toId);
    return new Link(fromId, toId, data, linkId);
  }

  function createUniqueLink(fromId, toId, data) {
    // TODO: Get rid of this method.
    var linkId = makeLinkId(fromId, toId);
    var isMultiEdge = multiEdges.hasOwnProperty(linkId);
    if (isMultiEdge || getLink(fromId, toId)) {
      if (!isMultiEdge) {
        multiEdges[linkId] = 0;
      }
      var suffix = '@' + (++multiEdges[linkId]);
      linkId = makeLinkId(fromId + suffix, toId + suffix);
    }

    return new Link(fromId, toId, data, linkId);
  }

  function getLinks(nodeId) {
    var node = getNode(nodeId);
    return node ? node.links : null;
  }

  function removeLink(link) {
    if (!link) {
      return false;
    }
    var idx = indexOfElementInArray(link, links);
    if (idx < 0) {
      return false;
    }

    enterModification();

    links.splice(idx, 1);

    var fromNode = getNode(link.fromId);
    var toNode = getNode(link.toId);

    if (fromNode) {
      idx = indexOfElementInArray(link, fromNode.links);
      if (idx >= 0) {
        fromNode.links.splice(idx, 1);
      }
    }

    if (toNode) {
      idx = indexOfElementInArray(link, toNode.links);
      if (idx >= 0) {
        toNode.links.splice(idx, 1);
      }
    }

    recordLinkChange(link, 'remove');

    exitModification();

    return true;
  }

  function getLink(fromNodeId, toNodeId) {
    // TODO: Use sorted links to speed this up
    var node = getNode(fromNodeId),
      i;
    if (!node || !node.links) {
      return null;
    }

    for (i = 0; i < node.links.length; ++i) {
      var link = node.links[i];
      if (link.fromId === fromNodeId && link.toId === toNodeId) {
        return link;
      }
    }

    return null; // no link.
  }

  function clear() {
    enterModification();
    forEachNode(function(node) {
      removeNode(node.id);
    });
    exitModification();
  }

  function forEachLink(callback) {
    var i, length;
    if (typeof callback === 'function') {
      for (i = 0, length = links.length; i < length; ++i) {
        callback(links[i]);
      }
    }
  }

  function forEachLinkedNode(nodeId, callback, oriented) {
    var node = getNode(nodeId);

    if (node && node.links && typeof callback === 'function') {
      if (oriented) {
        return forEachOrientedLink(node.links, nodeId, callback);
      } else {
        return forEachNonOrientedLink(node.links, nodeId, callback);
      }
    }
  }

  function forEachNonOrientedLink(links, nodeId, callback) {
    var quitFast;
    for (var i = 0; i < links.length; ++i) {
      var link = links[i];
      var linkedNodeId = link.fromId === nodeId ? link.toId : link.fromId;

      quitFast = callback(nodes[linkedNodeId], link);
      if (quitFast) {
        return true; // Client does not need more iterations. Break now.
      }
    }
  }

  function forEachOrientedLink(links, nodeId, callback) {
    var quitFast;
    for (var i = 0; i < links.length; ++i) {
      var link = links[i];
      if (link.fromId === nodeId) {
        quitFast = callback(nodes[link.toId], link);
        if (quitFast) {
          return true; // Client does not need more iterations. Break now.
        }
      }
    }
  }

  // we will not fire anything until users of this library explicitly call `on()`
  // method.
  function noop() {}

  // Enter, Exit modification allows bulk graph updates without firing events.
  function enterModificationReal() {
    suspendEvents += 1;
  }

  function exitModificationReal() {
    suspendEvents -= 1;
    if (suspendEvents === 0 && changes.length > 0) {
      graphPart.fire('changed', changes);
      changes.length = 0;
    }
  }

  function createNodeIterator() {
    // Object.keys iterator is 1.3x faster than `for in` loop.
    // See `https://github.com/anvaka/ngraph.graph/tree/bench-for-in-vs-obj-keys`
    // branch for perf test
    return Object.keys ? objectKeysIterator : forInIterator;
  }

  function objectKeysIterator(callback) {
    if (typeof callback !== 'function') {
      return;
    }

    var keys = Object.keys(nodes);
    for (var i = 0; i < keys.length; ++i) {
      if (callback(nodes[keys[i]])) {
        return true; // client doesn't want to proceed. Return.
      }
    }
  }

  function forInIterator(callback) {
    if (typeof callback !== 'function') {
      return;
    }
    var node;

    for (node in nodes) {
      if (callback(nodes[node])) {
        return true; // client doesn't want to proceed. Return.
      }
    }
  }
}

// need this for old browsers. Should this be a separate module?
function indexOfElementInArray(element, array) {
  if (!array) return -1;

  if (array.indexOf) {
    return array.indexOf(element);
  }

  var len = array.length,
    i;

  for (i = 0; i < len; i += 1) {
    if (array[i] === element) {
      return i;
    }
  }

  return -1;
}

/**
 * Internal structure to represent node;
 */
function Node(id, data) {
  this.id = id;
  this.links = null;
  this.data = data;
}

function addLinkToNode(node, link) {
  if (node.links) {
    node.links.push(link);
  } else {
    node.links = [link];
  }
}

/**
 * Internal structure to represent links;
 */
function Link(fromId, toId, data, id) {
  this.fromId = fromId;
  this.toId = toId;
  this.data = data;
  this.id = id;
}

function hashCode(str) {
  var hash = 0, i, chr, len;
  if (str.length == 0) return hash;
  for (i = 0, len = str.length; i < len; i++) {
    chr   = str.charCodeAt(i);
    hash  = ((hash << 5) - hash) + chr;
    hash |= 0; // Convert to 32bit integer
  }
  return hash;
}

function makeLinkId(fromId, toId) {
  return fromId.toString() + '👉 ' + toId.toString();
}

},{"ngraph.events":1}],13:[function(require,module,exports){
module.exports = merge;

/**
 * Augments `target` with properties in `options`. Does not override
 * target's properties if they are defined and matches expected type in 
 * options
 *
 * @returns {Object} merged object
 */
function merge(target, options) {
  var key;
  if (!target) { target = {}; }
  if (options) {
    for (key in options) {
      if (options.hasOwnProperty(key)) {
        var targetHasIt = target.hasOwnProperty(key),
            optionsValueType = typeof options[key],
            shouldReplace = !targetHasIt || (typeof target[key] !== optionsValueType);

        if (shouldReplace) {
          target[key] = options[key];
        } else if (optionsValueType === 'object') {
          // go deep, don't care about loops here, we are simple API!:
          target[key] = merge(target[key], options[key]);
        }
      }
    }
  }

  return target;
}

},{}],14:[function(require,module,exports){
module.exports = {
  Body: Body,
  Vector2d: Vector2d,
  Body3d: Body3d,
  Vector3d: Vector3d
};

function Body(x, y) {
  this.pos = new Vector2d(x, y);
  this.prevPos = new Vector2d(x, y);
  this.force = new Vector2d();
  this.velocity = new Vector2d();
  this.mass = 1;
}

Body.prototype.setPosition = function (x, y) {
  this.prevPos.x = this.pos.x = x;
  this.prevPos.y = this.pos.y = y;
};

function Vector2d(x, y) {
  if (x && typeof x !== 'number') {
    // could be another vector
    this.x = typeof x.x === 'number' ? x.x : 0;
    this.y = typeof x.y === 'number' ? x.y : 0;
  } else {
    this.x = typeof x === 'number' ? x : 0;
    this.y = typeof y === 'number' ? y : 0;
  }
}

Vector2d.prototype.reset = function () {
  this.x = this.y = 0;
};

function Body3d(x, y, z) {
  this.pos = new Vector3d(x, y, z);
  this.prevPos = new Vector3d(x, y, z);
  this.force = new Vector3d();
  this.velocity = new Vector3d();
  this.mass = 1;
}

Body3d.prototype.setPosition = function (x, y, z) {
  this.prevPos.x = this.pos.x = x;
  this.prevPos.y = this.pos.y = y;
  this.prevPos.z = this.pos.z = z;
};

function Vector3d(x, y, z) {
  if (x && typeof x !== 'number') {
    // could be another vector
    this.x = typeof x.x === 'number' ? x.x : 0;
    this.y = typeof x.y === 'number' ? x.y : 0;
    this.z = typeof x.z === 'number' ? x.z : 0;
  } else {
    this.x = typeof x === 'number' ? x : 0;
    this.y = typeof y === 'number' ? y : 0;
    this.z = typeof z === 'number' ? z : 0;
  }
};

Vector3d.prototype.reset = function () {
  this.x = this.y = this.z = 0;
};

},{}],15:[function(require,module,exports){
/**
 * Manages a simulation of physical forces acting on bodies and springs.
 */
module.exports = physicsSimulator;

function physicsSimulator(settings) {
  var Spring = require('./lib/spring');
  var expose = require('ngraph.expose');
  var merge = require('ngraph.merge');

  settings = merge(settings, {
      /**
       * Ideal length for links (springs in physical model).
       */
      springLength: 30,

      /**
       * Hook's law coefficient. 1 - solid spring.
       */
      springCoeff: 0.0008,

      /**
       * Coulomb's law coefficient. It's used to repel nodes thus should be negative
       * if you make it positive nodes start attract each other :).
       */
      gravity: -1.2,

      /**
       * Theta coefficient from Barnes Hut simulation. Ranged between (0, 1).
       * The closer it's to 1 the more nodes algorithm will have to go through.
       * Setting it to one makes Barnes Hut simulation no different from
       * brute-force forces calculation (each node is considered).
       */
      theta: 0.8,

      /**
       * Drag force coefficient. Used to slow down system, thus should be less than 1.
       * The closer it is to 0 the less tight system will be.
       */
      dragCoeff: 0.02,

      /**
       * Default time step (dt) for forces integration
       */
      timeStep : 20,

      /**
        * Maximum movement of the system which can be considered as stabilized
        */
      stableThreshold: 0.009
  });

  // We allow clients to override basic factory methods:
  var createQuadTree = settings.createQuadTree || require('ngraph.quadtreebh');
  var createBounds = settings.createBounds || require('./lib/bounds');
  var createDragForce = settings.createDragForce || require('./lib/dragForce');
  var createSpringForce = settings.createSpringForce || require('./lib/springForce');
  var integrate = settings.integrator || require('./lib/eulerIntegrator');
  var createBody = settings.createBody || require('./lib/createBody');

  var bodies = [], // Bodies in this simulation.
      springs = [], // Springs in this simulation.
      quadTree =  createQuadTree(settings),
      bounds = createBounds(bodies, settings),
      springForce = createSpringForce(settings),
      dragForce = createDragForce(settings);

  var publicApi = {
    /**
     * Array of bodies, registered with current simulator
     *
     * Note: To add new body, use addBody() method. This property is only
     * exposed for testing/performance purposes.
     */
    bodies: bodies,

    /**
     * Array of springs, registered with current simulator
     *
     * Note: To add new spring, use addSpring() method. This property is only
     * exposed for testing/performance purposes.
     */
    springs: springs,

    /**
     * Returns settings with which current simulator was initialized
     */
    settings: settings,

    /**
     * Performs one step of force simulation.
     *
     * @returns {boolean} true if system is considered stable; False otherwise.
     */
    step: function () {
      accumulateForces();
      var totalMovement = integrate(bodies, settings.timeStep);

      bounds.update();

      return totalMovement < settings.stableThreshold;
    },

    /**
     * Adds body to the system
     *
     * @param {ngraph.physics.primitives.Body} body physical body
     *
     * @returns {ngraph.physics.primitives.Body} added body
     */
    addBody: function (body) {
      if (!body) {
        throw new Error('Body is required');
      }
      bodies.push(body);

      return body;
    },

    /**
     * Adds body to the system at given position
     *
     * @param {Object} pos position of a body
     *
     * @returns {ngraph.physics.primitives.Body} added body
     */
    addBodyAt: function (pos) {
      if (!pos) {
        throw new Error('Body position is required');
      }
      var body = createBody(pos);
      bodies.push(body);

      return body;
    },

    /**
     * Removes body from the system
     *
     * @param {ngraph.physics.primitives.Body} body to remove
     *
     * @returns {Boolean} true if body found and removed. falsy otherwise;
     */
    removeBody: function (body) {
      if (!body) { return; }

      var idx = bodies.indexOf(body);
      if (idx < 0) { return; }

      bodies.splice(idx, 1);
      if (bodies.length === 0) {
        bounds.reset();
      }
      return true;
    },

    /**
     * Adds a spring to this simulation.
     *
     * @returns {Object} - a handle for a spring. If you want to later remove
     * spring pass it to removeSpring() method.
     */
    addSpring: function (body1, body2, springLength, springWeight, springCoefficient) {
      if (!body1 || !body2) {
        throw new Error('Cannot add null spring to force simulator');
      }

      if (typeof springLength !== 'number') {
        springLength = -1; // assume global configuration
      }

      var spring = new Spring(body1, body2, springLength, springCoefficient >= 0 ? springCoefficient : -1, springWeight);
      springs.push(spring);

      // TODO: could mark simulator as dirty.
      return spring;
    },

    /**
     * Removes spring from the system
     *
     * @param {Object} spring to remove. Spring is an object returned by addSpring
     *
     * @returns {Boolean} true if spring found and removed. falsy otherwise;
     */
    removeSpring: function (spring) {
      if (!spring) { return; }
      var idx = springs.indexOf(spring);
      if (idx > -1) {
        springs.splice(idx, 1);
        return true;
      }
    },

    getBestNewBodyPosition: function (neighbors) {
      return bounds.getBestNewPosition(neighbors);
    },

    /**
     * Returns bounding box which covers all bodies
     */
    getBBox: function () {
      return bounds.box;
    },

    gravity: function (value) {
      if (value !== undefined) {
        settings.gravity = value;
        quadTree.options({gravity: value});
        return this;
      } else {
        return settings.gravity;
      }
    },

    theta: function (value) {
      if (value !== undefined) {
        settings.theta = value;
        quadTree.options({theta: value});
        return this;
      } else {
        return settings.theta;
      }
    }
  };

  // allow settings modification via public API:
  expose(settings, publicApi);

  return publicApi;

  function accumulateForces() {
    // Accumulate forces acting on bodies.
    var body,
        i = bodies.length;

    if (i) {
      // only add bodies if there the array is not empty:
      quadTree.insertBodies(bodies); // performance: O(n * log n)
      while (i--) {
        body = bodies[i];
        // If body is pinned there is no point updating its forces - it should
        // never move:
        if (!body.isPinned) {
          body.force.reset();

          quadTree.updateBodyForce(body);
          dragForce.update(body);
        }
      }
    }

    i = springs.length;
    while(i--) {
      springForce.update(springs[i]);
    }
  }
};

},{"./lib/bounds":16,"./lib/createBody":17,"./lib/dragForce":18,"./lib/eulerIntegrator":19,"./lib/spring":20,"./lib/springForce":21,"ngraph.expose":2,"ngraph.merge":13,"ngraph.quadtreebh":22}],16:[function(require,module,exports){
arguments[4][5][0].apply(exports,arguments)
},{"dup":5,"ngraph.random":26}],17:[function(require,module,exports){
arguments[4][6][0].apply(exports,arguments)
},{"dup":6,"ngraph.physics.primitives":14}],18:[function(require,module,exports){
arguments[4][7][0].apply(exports,arguments)
},{"dup":7,"ngraph.expose":2,"ngraph.merge":13}],19:[function(require,module,exports){
/**
 * Performs forces integration, using given timestep. Uses Euler method to solve
 * differential equation (http://en.wikipedia.org/wiki/Euler_method ).
 *
 * @returns {Number} squared distance of total position updates.
 */

module.exports = integrate;

function integrate(bodies, timeStep) {
  var dx = 0, tx = 0,
      dy = 0, ty = 0,
      i,
      max = bodies.length;

  for (i = 0; i < max; ++i) {
    var body = bodies[i],
        coeff = timeStep / body.mass;

    body.velocity.x += coeff * body.force.x;
    body.velocity.y += coeff * body.force.y;
    var vx = body.velocity.x,
        vy = body.velocity.y,
        v = Math.sqrt(vx * vx + vy * vy);

    if (v > 1) {
      body.velocity.x = vx / v;
      body.velocity.y = vy / v;
    }

    dx = timeStep * body.velocity.x;
    dy = timeStep * body.velocity.y;

    body.pos.x += dx;
    body.pos.y += dy;

    tx += Math.abs(dx); ty += Math.abs(dy);
  }

  return (tx * tx + ty * ty)/bodies.length;
}

},{}],20:[function(require,module,exports){
arguments[4][9][0].apply(exports,arguments)
},{"dup":9}],21:[function(require,module,exports){
arguments[4][10][0].apply(exports,arguments)
},{"dup":10,"ngraph.expose":2,"ngraph.merge":13,"ngraph.random":26}],22:[function(require,module,exports){
/**
 * This is Barnes Hut simulation algorithm for 2d case. Implementation
 * is highly optimized (avoids recusion and gc pressure)
 *
 * http://www.cs.princeton.edu/courses/archive/fall03/cs126/assignments/barnes-hut.html
 */

module.exports = function(options) {
  options = options || {};
  options.gravity = typeof options.gravity === 'number' ? options.gravity : -1;
  options.theta = typeof options.theta === 'number' ? options.theta : 0.8;

  // we require deterministic randomness here
  var random = require('ngraph.random').random(1984),
    Node = require('./node'),
    InsertStack = require('./insertStack'),
    isSamePosition = require('./isSamePosition');

  var gravity = options.gravity,
    updateQueue = [],
    insertStack = new InsertStack(),
    theta = options.theta,

    nodesCache = [],
    currentInCache = 0,
    newNode = function() {
      // To avoid pressure on GC we reuse nodes.
      var node = nodesCache[currentInCache];
      if (node) {
        node.quad0 = null;
        node.quad1 = null;
        node.quad2 = null;
        node.quad3 = null;
        node.body = null;
        node.mass = node.massX = node.massY = 0;
        node.left = node.right = node.top = node.bottom = 0;
      } else {
        node = new Node();
        nodesCache[currentInCache] = node;
      }

      ++currentInCache;
      return node;
    },

    root = newNode(),

    // Inserts body to the tree
    insert = function(newBody) {
      insertStack.reset();
      insertStack.push(root, newBody);

      while (!insertStack.isEmpty()) {
        var stackItem = insertStack.pop(),
          node = stackItem.node,
          body = stackItem.body;

        if (!node.body) {
          // This is internal node. Update the total mass of the node and center-of-mass.
          var x = body.pos.x;
          var y = body.pos.y;
          node.mass = node.mass + body.mass;
          node.massX = node.massX + body.mass * x;
          node.massY = node.massY + body.mass * y;

          // Recursively insert the body in the appropriate quadrant.
          // But first find the appropriate quadrant.
          var quadIdx = 0, // Assume we are in the 0's quad.
            left = node.left,
            right = (node.right + left) / 2,
            top = node.top,
            bottom = (node.bottom + top) / 2;

          if (x > right) { // somewhere in the eastern part.
            quadIdx = quadIdx + 1;
            var oldLeft = left;
            left = right;
            right = right + (right - oldLeft);
          }
          if (y > bottom) { // and in south.
            quadIdx = quadIdx + 2;
            var oldTop = top;
            top = bottom;
            bottom = bottom + (bottom - oldTop);
          }

          var child = getChild(node, quadIdx);
          if (!child) {
            // The node is internal but this quadrant is not taken. Add
            // subnode to it.
            child = newNode();
            child.left = left;
            child.top = top;
            child.right = right;
            child.bottom = bottom;
            child.body = body;

            setChild(node, quadIdx, child);
          } else {
            // continue searching in this quadrant.
            insertStack.push(child, body);
          }
        } else {
          // We are trying to add to the leaf node.
          // We have to convert current leaf into internal node
          // and continue adding two nodes.
          var oldBody = node.body;
          node.body = null; // internal nodes do not cary bodies

          if (isSamePosition(oldBody.pos, body.pos)) {
            // Prevent infinite subdivision by bumping one node
            // anywhere in this quadrant
            var retriesCount = 3;
            do {
              var offset = random.nextDouble();
              var dx = (node.right - node.left) * offset;
              var dy = (node.bottom - node.top) * offset;

              oldBody.pos.x = node.left + dx;
              oldBody.pos.y = node.top + dy;
              retriesCount -= 1;
              // Make sure we don't bump it out of the box. If we do, next iteration should fix it
            } while (retriesCount > 0 && isSamePosition(oldBody.pos, body.pos));

            if (retriesCount === 0 && isSamePosition(oldBody.pos, body.pos)) {
              // This is very bad, we ran out of precision.
              // if we do not return from the method we'll get into
              // infinite loop here. So we sacrifice correctness of layout, and keep the app running
              // Next layout iteration should get larger bounding box in the first step and fix this
              return;
            }
          }
          // Next iteration should subdivide node further.
          insertStack.push(node, oldBody);
          insertStack.push(node, body);
        }
      }
    },

    update = function(sourceBody) {
      var queue = updateQueue,
        v,
        dx,
        dy,
        r, fx = 0,
        fy = 0,
        queueLength = 1,
        shiftIdx = 0,
        pushIdx = 1;

      queue[0] = root;

      while (queueLength) {
        var node = queue[shiftIdx],
          body = node.body;

        queueLength -= 1;
        shiftIdx += 1;
        var differentBody = (body !== sourceBody);
        if (body && differentBody) {
          // If the current node is a leaf node (and it is not source body),
          // calculate the force exerted by the current node on body, and add this
          // amount to body's net force.
          dx = body.pos.x - sourceBody.pos.x;
          dy = body.pos.y - sourceBody.pos.y;
          r = Math.sqrt(dx * dx + dy * dy);

          if (r === 0) {
            // Poor man's protection against zero distance.
            dx = (random.nextDouble() - 0.5) / 50;
            dy = (random.nextDouble() - 0.5) / 50;
            r = Math.sqrt(dx * dx + dy * dy);
          }

          // This is standard gravition force calculation but we divide
          // by r^3 to save two operations when normalizing force vector.
          v = gravity * body.mass * sourceBody.mass / (r * r * r);
          fx += v * dx;
          fy += v * dy;
        } else if (differentBody) {
          // Otherwise, calculate the ratio s / r,  where s is the width of the region
          // represented by the internal node, and r is the distance between the body
          // and the node's center-of-mass
          dx = node.massX / node.mass - sourceBody.pos.x;
          dy = node.massY / node.mass - sourceBody.pos.y;
          r = Math.sqrt(dx * dx + dy * dy);

          if (r === 0) {
            // Sorry about code duplucation. I don't want to create many functions
            // right away. Just want to see performance first.
            dx = (random.nextDouble() - 0.5) / 50;
            dy = (random.nextDouble() - 0.5) / 50;
            r = Math.sqrt(dx * dx + dy * dy);
          }
          // If s / r < θ, treat this internal node as a single body, and calculate the
          // force it exerts on sourceBody, and add this amount to sourceBody's net force.
          if ((node.right - node.left) / r < theta) {
            // in the if statement above we consider node's width only
            // because the region was squarified during tree creation.
            // Thus there is no difference between using width or height.
            v = gravity * node.mass * sourceBody.mass / (r * r * r);
            fx += v * dx;
            fy += v * dy;
          } else {
            // Otherwise, run the procedure recursively on each of the current node's children.

            // I intentionally unfolded this loop, to save several CPU cycles.
            if (node.quad0) {
              queue[pushIdx] = node.quad0;
              queueLength += 1;
              pushIdx += 1;
            }
            if (node.quad1) {
              queue[pushIdx] = node.quad1;
              queueLength += 1;
              pushIdx += 1;
            }
            if (node.quad2) {
              queue[pushIdx] = node.quad2;
              queueLength += 1;
              pushIdx += 1;
            }
            if (node.quad3) {
              queue[pushIdx] = node.quad3;
              queueLength += 1;
              pushIdx += 1;
            }
          }
        }
      }

      sourceBody.force.x += fx;
      sourceBody.force.y += fy;
    },

    insertBodies = function(bodies) {
      var x1 = Number.MAX_VALUE,
        y1 = Number.MAX_VALUE,
        x2 = Number.MIN_VALUE,
        y2 = Number.MIN_VALUE,
        i,
        max = bodies.length;

      // To reduce quad tree depth we are looking for exact bounding box of all particles.
      i = max;
      while (i--) {
        var x = bodies[i].pos.x;
        var y = bodies[i].pos.y;
        if (x < x1) {
          x1 = x;
        }
        if (x > x2) {
          x2 = x;
        }
        if (y < y1) {
          y1 = y;
        }
        if (y > y2) {
          y2 = y;
        }
      }

      // Squarify the bounds.
      var dx = x2 - x1,
        dy = y2 - y1;
      if (dx > dy) {
        y2 = y1 + dx;
      } else {
        x2 = x1 + dy;
      }

      currentInCache = 0;
      root = newNode();
      root.left = x1;
      root.right = x2;
      root.top = y1;
      root.bottom = y2;

      i = max - 1;
      if (i > 0) {
        root.body = bodies[i];
      }
      while (i--) {
        insert(bodies[i], root);
      }
    };

  return {
    insertBodies: insertBodies,
    updateBodyForce: update,
    options: function(newOptions) {
      if (newOptions) {
        if (typeof newOptions.gravity === 'number') {
          gravity = newOptions.gravity;
        }
        if (typeof newOptions.theta === 'number') {
          theta = newOptions.theta;
        }

        return this;
      }

      return {
        gravity: gravity,
        theta: theta
      };
    }
  };
};

function getChild(node, idx) {
  if (idx === 0) return node.quad0;
  if (idx === 1) return node.quad1;
  if (idx === 2) return node.quad2;
  if (idx === 3) return node.quad3;
  return null;
}

function setChild(node, idx, child) {
  if (idx === 0) node.quad0 = child;
  else if (idx === 1) node.quad1 = child;
  else if (idx === 2) node.quad2 = child;
  else if (idx === 3) node.quad3 = child;
}

},{"./insertStack":23,"./isSamePosition":24,"./node":25,"ngraph.random":26}],23:[function(require,module,exports){
module.exports = InsertStack;

/**
 * Our implmentation of QuadTree is non-recursive to avoid GC hit
 * This data structure represent stack of elements
 * which we are trying to insert into quad tree.
 */
function InsertStack () {
    this.stack = [];
    this.popIdx = 0;
}

InsertStack.prototype = {
    isEmpty: function() {
        return this.popIdx === 0;
    },
    push: function (node, body) {
        var item = this.stack[this.popIdx];
        if (!item) {
            // we are trying to avoid memory pressue: create new element
            // only when absolutely necessary
            this.stack[this.popIdx] = new InsertStackElement(node, body);
        } else {
            item.node = node;
            item.body = body;
        }
        ++this.popIdx;
    },
    pop: function () {
        if (this.popIdx > 0) {
            return this.stack[--this.popIdx];
        }
    },
    reset: function () {
        this.popIdx = 0;
    }
};

function InsertStackElement(node, body) {
    this.node = node; // QuadTree node
    this.body = body; // physical body which needs to be inserted to node
}

},{}],24:[function(require,module,exports){
module.exports = function isSamePosition(point1, point2) {
    var dx = Math.abs(point1.x - point2.x);
    var dy = Math.abs(point1.y - point2.y);

    return (dx < 1e-8 && dy < 1e-8);
};

},{}],25:[function(require,module,exports){
/**
 * Internal data structure to represent 2D QuadTree node
 */
module.exports = function Node() {
  // body stored inside this node. In quad tree only leaf nodes (by construction)
  // contain boides:
  this.body = null;

  // Child nodes are stored in quads. Each quad is presented by number:
  // 0 | 1
  // -----
  // 2 | 3
  this.quad0 = null;
  this.quad1 = null;
  this.quad2 = null;
  this.quad3 = null;

  // Total mass of current node
  this.mass = 0;

  // Center of mass coordinates
  this.massX = 0;
  this.massY = 0;

  // bounding box coordinates
  this.left = 0;
  this.top = 0;
  this.bottom = 0;
  this.right = 0;
};

},{}],26:[function(require,module,exports){
arguments[4][11][0].apply(exports,arguments)
},{"dup":11}],27:[function(require,module,exports){
/**
 * This is Barnes Hut simulation algorithm for 2d case. Implementation
 * is highly optimized (avoids recusion and gc pressure)
 *
 * http://www.cs.princeton.edu/courses/archive/fall03/cs126/assignments/barnes-hut.html
 */

module.exports = function(options) {
  options = options || {};
  options.gravity = typeof options.gravity === 'number' ? options.gravity : -1;
  options.theta = typeof options.theta === 'number' ? options.theta : 0.8;

  // we require deterministic randomness here
  var random = require('ngraph.random').random(1984),
    Node = require('./node'),
    InsertStack = require('./insertStack'),
    isSamePosition = require('./isSamePosition');

  var gravity = options.gravity,
    updateQueue = [],
    insertStack = new InsertStack(),
    theta = options.theta,

    nodesCache = [],
    currentInCache = 0,
    root = newNode();

  return {
    insertBodies: insertBodies,
    /**
     * Gets root node if its present
     */
    getRoot: function() {
      return root;
    },
    updateBodyForce: update,
    options: function(newOptions) {
      if (newOptions) {
        if (typeof newOptions.gravity === 'number') {
          gravity = newOptions.gravity;
        }
        if (typeof newOptions.theta === 'number') {
          theta = newOptions.theta;
        }

        return this;
      }

      return {
        gravity: gravity,
        theta: theta
      };
    }
  };

  function newNode() {
    // To avoid pressure on GC we reuse nodes.
    var node = nodesCache[currentInCache];
    if (node) {
      node.quad0 = null;
      node.quad1 = null;
      node.quad2 = null;
      node.quad3 = null;
      node.body = null;
      node.mass = node.massX = node.massY = 0;
      node.left = node.right = node.top = node.bottom = 0;
    } else {
      node = new Node();
      nodesCache[currentInCache] = node;
    }

    ++currentInCache;
    return node;
  }

  function update(sourceBody) {
    var queue = updateQueue,
      v,
      dx,
      dy,
      r, fx = 0,
      fy = 0,
      queueLength = 1,
      shiftIdx = 0,
      pushIdx = 1;

    queue[0] = root;

    while (queueLength) {
      var node = queue[shiftIdx],
        body = node.body;

      queueLength -= 1;
      shiftIdx += 1;
      var differentBody = (body !== sourceBody);
      if (body && differentBody) {
        // If the current node is a leaf node (and it is not source body),
        // calculate the force exerted by the current node on body, and add this
        // amount to body's net force.
        dx = body.pos.x - sourceBody.pos.x;
        dy = body.pos.y - sourceBody.pos.y;
        r = Math.sqrt(dx * dx + dy * dy);

        if (r === 0) {
          // Poor man's protection against zero distance.
          dx = (random.nextDouble() - 0.5) / 50;
          dy = (random.nextDouble() - 0.5) / 50;
          r = Math.sqrt(dx * dx + dy * dy);
        }

        // This is standard gravition force calculation but we divide
        // by r^3 to save two operations when normalizing force vector.
        v = gravity * body.mass * sourceBody.mass / (r * r * r);
        fx += v * dx;
        fy += v * dy;
      } else if (differentBody) {
        // Otherwise, calculate the ratio s / r,  where s is the width of the region
        // represented by the internal node, and r is the distance between the body
        // and the node's center-of-mass
        dx = node.massX / node.mass - sourceBody.pos.x;
        dy = node.massY / node.mass - sourceBody.pos.y;
        r = Math.sqrt(dx * dx + dy * dy);

        if (r === 0) {
          // Sorry about code duplucation. I don't want to create many functions
          // right away. Just want to see performance first.
          dx = (random.nextDouble() - 0.5) / 50;
          dy = (random.nextDouble() - 0.5) / 50;
          r = Math.sqrt(dx * dx + dy * dy);
        }
        // If s / r < θ, treat this internal node as a single body, and calculate the
        // force it exerts on sourceBody, and add this amount to sourceBody's net force.
        if ((node.right - node.left) / r < theta) {
          // in the if statement above we consider node's width only
          // because the region was squarified during tree creation.
          // Thus there is no difference between using width or height.
          v = gravity * node.mass * sourceBody.mass / (r * r * r);
          fx += v * dx;
          fy += v * dy;
        } else {
          // Otherwise, run the procedure recursively on each of the current node's children.

          // I intentionally unfolded this loop, to save several CPU cycles.
          if (node.quad0) {
            queue[pushIdx] = node.quad0;
            queueLength += 1;
            pushIdx += 1;
          }
          if (node.quad1) {
            queue[pushIdx] = node.quad1;
            queueLength += 1;
            pushIdx += 1;
          }
          if (node.quad2) {
            queue[pushIdx] = node.quad2;
            queueLength += 1;
            pushIdx += 1;
          }
          if (node.quad3) {
            queue[pushIdx] = node.quad3;
            queueLength += 1;
            pushIdx += 1;
          }
        }
      }
    }

    sourceBody.force.x += fx;
    sourceBody.force.y += fy;
  }

  function insertBodies(bodies) {
    var x1 = Number.MAX_VALUE,
      y1 = Number.MAX_VALUE,
      x2 = Number.MIN_VALUE,
      y2 = Number.MIN_VALUE,
      i,
      max = bodies.length;

    // To reduce quad tree depth we are looking for exact bounding box of all particles.
    i = max;
    while (i--) {
      var x = bodies[i].pos.x;
      var y = bodies[i].pos.y;
      if (x < x1) {
        x1 = x;
      }
      if (x > x2) {
        x2 = x;
      }
      if (y < y1) {
        y1 = y;
      }
      if (y > y2) {
        y2 = y;
      }
    }

    // Squarify the bounds.
    var dx = x2 - x1,
      dy = y2 - y1;
    if (dx > dy) {
      y2 = y1 + dx;
    } else {
      x2 = x1 + dy;
    }

    currentInCache = 0;
    root = newNode();
    root.left = x1;
    root.right = x2;
    root.top = y1;
    root.bottom = y2;

    i = max - 1;
    if (i >= 0) {
      root.body = bodies[i];
    }
    while (i--) {
      insert(bodies[i], root);
    }
  }

  function insert(newBody) {
    insertStack.reset();
    insertStack.push(root, newBody);

    while (!insertStack.isEmpty()) {
      var stackItem = insertStack.pop(),
        node = stackItem.node,
        body = stackItem.body;

      if (!node.body) {
        // This is internal node. Update the total mass of the node and center-of-mass.
        var x = body.pos.x;
        var y = body.pos.y;
        node.mass = node.mass + body.mass;
        node.massX = node.massX + body.mass * x;
        node.massY = node.massY + body.mass * y;

        // Recursively insert the body in the appropriate quadrant.
        // But first find the appropriate quadrant.
        var quadIdx = 0, // Assume we are in the 0's quad.
          left = node.left,
          right = (node.right + left) / 2,
          top = node.top,
          bottom = (node.bottom + top) / 2;

        if (x > right) { // somewhere in the eastern part.
          quadIdx = quadIdx + 1;
          left = right;
          right = node.right;
        }
        if (y > bottom) { // and in south.
          quadIdx = quadIdx + 2;
          top = bottom;
          bottom = node.bottom;
        }

        var child = getChild(node, quadIdx);
        if (!child) {
          // The node is internal but this quadrant is not taken. Add
          // subnode to it.
          child = newNode();
          child.left = left;
          child.top = top;
          child.right = right;
          child.bottom = bottom;
          child.body = body;

          setChild(node, quadIdx, child);
        } else {
          // continue searching in this quadrant.
          insertStack.push(child, body);
        }
      } else {
        // We are trying to add to the leaf node.
        // We have to convert current leaf into internal node
        // and continue adding two nodes.
        var oldBody = node.body;
        node.body = null; // internal nodes do not cary bodies

        if (isSamePosition(oldBody.pos, body.pos)) {
          // Prevent infinite subdivision by bumping one node
          // anywhere in this quadrant
          var retriesCount = 3;
          do {
            var offset = random.nextDouble();
            var dx = (node.right - node.left) * offset;
            var dy = (node.bottom - node.top) * offset;

            oldBody.pos.x = node.left + dx;
            oldBody.pos.y = node.top + dy;
            retriesCount -= 1;
            // Make sure we don't bump it out of the box. If we do, next iteration should fix it
          } while (retriesCount > 0 && isSamePosition(oldBody.pos, body.pos));

          if (retriesCount === 0 && isSamePosition(oldBody.pos, body.pos)) {
            // This is very bad, we ran out of precision.
            // if we do not return from the method we'll get into
            // infinite loop here. So we sacrifice correctness of layout, and keep the app running
            // Next layout iteration should get larger bounding box in the first step and fix this
            return;
          }
        }
        // Next iteration should subdivide node further.
        insertStack.push(node, oldBody);
        insertStack.push(node, body);
      }
    }
  }
};

function getChild(node, idx) {
  if (idx === 0) return node.quad0;
  if (idx === 1) return node.quad1;
  if (idx === 2) return node.quad2;
  if (idx === 3) return node.quad3;
  return null;
}

function setChild(node, idx, child) {
  if (idx === 0) node.quad0 = child;
  else if (idx === 1) node.quad1 = child;
  else if (idx === 2) node.quad2 = child;
  else if (idx === 3) node.quad3 = child;
}

},{"./insertStack":28,"./isSamePosition":29,"./node":30,"ngraph.random":31}],28:[function(require,module,exports){
arguments[4][23][0].apply(exports,arguments)
},{"dup":23}],29:[function(require,module,exports){
arguments[4][24][0].apply(exports,arguments)
},{"dup":24}],30:[function(require,module,exports){
arguments[4][25][0].apply(exports,arguments)
},{"dup":25}],31:[function(require,module,exports){
arguments[4][11][0].apply(exports,arguments)
},{"dup":11}],32:[function(require,module,exports){
module.exports.main = function () {

  if (typeof window === 'undefined') {
    console.log("In Node.js")
  } else {
    console.log("In Browser")
  }
  var graph = require('ngraph.graph')();

  // TODO: Separate the UI from the business logic
  var createNodes = require('./lib/mock_data.js')(graph)
  var layout = createLayout(graph);

//ILAP:  var createPixiGraphics = require('ngraph.pixi');
//ILAP:  var pixiGraphics = createPixiGraphics(graph, layout);
  // setup our custom looking nodes and links:
//ILAP:  pixiGraphics.createNodeUI(require('./lib/createNodeUI'))
//ILAP:    .renderNode(require('./lib/renderNode'))
//ILAP:    .createLinkUI(require('./lib/createLinkUI'))
//ILAP:    .renderLink(require('./lib/renderLink'));

//ILAP:  console.log(pixiGraphics.graphGraphics)
//ILAP:  pixiGraphics.graphGraphics.scale.x = 0.35

//ILAP:  pixiGraphics.graphGraphics.scale.y = 0.35

  //layout = pixiGraphics.layout;
  //var node = graph.getNode(tx) 
  //layout.pinNode(node,true);

  // begin animation loop:
  console.log("AAAA:" + JSON.stringify(createNodes))
  createNodes(0)
//ILAP:  pixiGraphics.run();

}

function createLayout(graph) {
  var layout = require('ngraph.forcelayout'),
      physics = require('ngraph.physics.simulator');

  return layout(graph, physics({
          springLength: 80,
          springCoeff: 0.0002,
          dragCoeff: 0.0002,
          gravity: -30,
          theta: 0.7
        }));
}


},{"./lib/mock_data.js":33,"ngraph.forcelayout":3,"ngraph.graph":12,"ngraph.physics.simulator":15}],33:[function(require,module,exports){
/**
 * @file
 * Provides random nodes and links that simulates transactions
 *
 * 
 */

module.exports = function nodes (graph) {
  
  var result = function createNode (id) {
  
    if ( id === undefined) {
      id = 0
    }
  
    var is = getRandomExp(1, 12, 16)
    var os = getRandomExp(2, 24)

    var tx = graph.addNode('t' + id, { type: 't', value: getRandomExp(100, 1000000, 1000 ) })

    console.log("In: %d, Out: %d, Txs: %s", is, os, JSON.stringify(tx))

    var io = {}
    var rn = 0 
    for ( var i = 0; i < is; i++) {
      rn = getRandomExp(10, 1000000, 400)   
      io = addNode(graph, 'i', i + id, rn)  
      // io = graph.addNode('i' + i + id, { type: 'i', value: getRandomExp(10, 1000000, 400) })
      graph.addLink(io.id, tx.id)
    }
    for ( var i = 0; i < os; i++) {
      io = graph.addNode('o' + i + id, { type: 'o', value: getRandomExp(10, 1000000, 400) })
      graph.addLink(tx.id, io.id)
    }
  
    if (id < 100) {
      setTimeout(function () {
        id++
        createNode(id.toString() )
      }, getRandomExp(500, 5000, 1000))
    }
  }
  
  return result
}

/**
 * Construct a node based on its type
 */
var addNode = function (graph, type, id, value) {
  nodeId = type + id
  data = { type: type, value: value }
  console.log("Node ID: %s, data: %s", nodeId, JSON.stringify(data))
  return graph.addNode(nodeId, data)    
}

/**
 * Generates a random integer between min and max.
 *
 * If mean is not null then it generates random number from the exponential distribution with the specified mean
 * Otherwise it only generates a random number between min and max value.
 *
 * @param {Integer} min
 *   Minimum value of the random number.
 * @param {Integer} max 
 *   Maximum value of the random number.
 * @param {Integer} mean 
 *   Mean value for the exponention random number.
 * 
 * @return
 *   the generated random number or -1 on any error.
 */
function getRandomExp(min, max, mean) {

  min = Math.ceil(min);
  max = Math.floor(max);
  result = -1

  if (typeof mean === 'undefined') {
     result = Math.floor(Math.random() * (max - min)) + min; 
  } else {
     result = -Math.log(Math.random()) * mean 
     result = result > max ? max : result <= min ? min : result
     result = Math.floor(result)
  }
  
  return result
}

},{}]},{},[32])(32)
});

//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJub2RlX21vZHVsZXMvbmdyYXBoLmV2ZW50cy9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGguZXhwb3NlL2luZGV4LmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5mb3JjZWxheW91dC9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGguZm9yY2VsYXlvdXQvbm9kZV9tb2R1bGVzL25ncmFwaC5waHlzaWNzLnNpbXVsYXRvci9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGguZm9yY2VsYXlvdXQvbm9kZV9tb2R1bGVzL25ncmFwaC5waHlzaWNzLnNpbXVsYXRvci9saWIvYm91bmRzLmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5mb3JjZWxheW91dC9ub2RlX21vZHVsZXMvbmdyYXBoLnBoeXNpY3Muc2ltdWxhdG9yL2xpYi9jcmVhdGVCb2R5LmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5mb3JjZWxheW91dC9ub2RlX21vZHVsZXMvbmdyYXBoLnBoeXNpY3Muc2ltdWxhdG9yL2xpYi9kcmFnRm9yY2UuanMiLCJub2RlX21vZHVsZXMvbmdyYXBoLmZvcmNlbGF5b3V0L25vZGVfbW9kdWxlcy9uZ3JhcGgucGh5c2ljcy5zaW11bGF0b3IvbGliL2V1bGVySW50ZWdyYXRvci5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGguZm9yY2VsYXlvdXQvbm9kZV9tb2R1bGVzL25ncmFwaC5waHlzaWNzLnNpbXVsYXRvci9saWIvc3ByaW5nLmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5mb3JjZWxheW91dC9ub2RlX21vZHVsZXMvbmdyYXBoLnBoeXNpY3Muc2ltdWxhdG9yL2xpYi9zcHJpbmdGb3JjZS5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGguZm9yY2VsYXlvdXQvbm9kZV9tb2R1bGVzL25ncmFwaC5yYW5kb20vaW5kZXguanMiLCJub2RlX21vZHVsZXMvbmdyYXBoLmdyYXBoL2luZGV4LmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5tZXJnZS9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGgucGh5c2ljcy5wcmltaXRpdmVzL2luZGV4LmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5waHlzaWNzLnNpbXVsYXRvci9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGgucGh5c2ljcy5zaW11bGF0b3IvbGliL2V1bGVySW50ZWdyYXRvci5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGgucGh5c2ljcy5zaW11bGF0b3Ivbm9kZV9tb2R1bGVzL25ncmFwaC5xdWFkdHJlZWJoL2luZGV4LmpzIiwibm9kZV9tb2R1bGVzL25ncmFwaC5waHlzaWNzLnNpbXVsYXRvci9ub2RlX21vZHVsZXMvbmdyYXBoLnF1YWR0cmVlYmgvaW5zZXJ0U3RhY2suanMiLCJub2RlX21vZHVsZXMvbmdyYXBoLnBoeXNpY3Muc2ltdWxhdG9yL25vZGVfbW9kdWxlcy9uZ3JhcGgucXVhZHRyZWViaC9pc1NhbWVQb3NpdGlvbi5qcyIsIm5vZGVfbW9kdWxlcy9uZ3JhcGgucGh5c2ljcy5zaW11bGF0b3Ivbm9kZV9tb2R1bGVzL25ncmFwaC5xdWFkdHJlZWJoL25vZGUuanMiLCJub2RlX21vZHVsZXMvbmdyYXBoLnF1YWR0cmVlYmgvaW5kZXguanMiLCJzcmMvYXBwLmpzIiwic3JjL2xpYi9tb2NrX2RhdGEuanMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7QUNBQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3hGQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDNUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzlXQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMzUUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2hGQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDTEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDM0JBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzdDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDZEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2xEQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3JGQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3psQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMvQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2pFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7Ozs7Ozs7QUNsUUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOzs7Ozs7QUN6Q0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDcFVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNOQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7OztBQzlCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7Ozs7Ozs7OztBQ3ZVQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDbERBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbigpe2Z1bmN0aW9uIHIoZSxuLHQpe2Z1bmN0aW9uIG8oaSxmKXtpZighbltpXSl7aWYoIWVbaV0pe3ZhciBjPVwiZnVuY3Rpb25cIj09dHlwZW9mIHJlcXVpcmUmJnJlcXVpcmU7aWYoIWYmJmMpcmV0dXJuIGMoaSwhMCk7aWYodSlyZXR1cm4gdShpLCEwKTt2YXIgYT1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK2krXCInXCIpO3Rocm93IGEuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixhfXZhciBwPW5baV09e2V4cG9ydHM6e319O2VbaV1bMF0uY2FsbChwLmV4cG9ydHMsZnVuY3Rpb24ocil7dmFyIG49ZVtpXVsxXVtyXTtyZXR1cm4gbyhufHxyKX0scCxwLmV4cG9ydHMscixlLG4sdCl9cmV0dXJuIG5baV0uZXhwb3J0c31mb3IodmFyIHU9XCJmdW5jdGlvblwiPT10eXBlb2YgcmVxdWlyZSYmcmVxdWlyZSxpPTA7aTx0Lmxlbmd0aDtpKyspbyh0W2ldKTtyZXR1cm4gb31yZXR1cm4gcn0pKCkiLCJtb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uKHN1YmplY3QpIHtcbiAgdmFsaWRhdGVTdWJqZWN0KHN1YmplY3QpO1xuXG4gIHZhciBldmVudHNTdG9yYWdlID0gY3JlYXRlRXZlbnRzU3RvcmFnZShzdWJqZWN0KTtcbiAgc3ViamVjdC5vbiA9IGV2ZW50c1N0b3JhZ2Uub247XG4gIHN1YmplY3Qub2ZmID0gZXZlbnRzU3RvcmFnZS5vZmY7XG4gIHN1YmplY3QuZmlyZSA9IGV2ZW50c1N0b3JhZ2UuZmlyZTtcbiAgcmV0dXJuIHN1YmplY3Q7XG59O1xuXG5mdW5jdGlvbiBjcmVhdGVFdmVudHNTdG9yYWdlKHN1YmplY3QpIHtcbiAgLy8gU3RvcmUgYWxsIGV2ZW50IGxpc3RlbmVycyB0byB0aGlzIGhhc2guIEtleSBpcyBldmVudCBuYW1lLCB2YWx1ZSBpcyBhcnJheVxuICAvLyBvZiBjYWxsYmFjayByZWNvcmRzLlxuICAvL1xuICAvLyBBIGNhbGxiYWNrIHJlY29yZCBjb25zaXN0cyBvZiBjYWxsYmFjayBmdW5jdGlvbiBhbmQgaXRzIG9wdGlvbmFsIGNvbnRleHQ6XG4gIC8vIHsgJ2V2ZW50TmFtZScgPT4gW3tjYWxsYmFjazogZnVuY3Rpb24sIGN0eDogb2JqZWN0fV0gfVxuICB2YXIgcmVnaXN0ZXJlZEV2ZW50cyA9IE9iamVjdC5jcmVhdGUobnVsbCk7XG5cbiAgcmV0dXJuIHtcbiAgICBvbjogZnVuY3Rpb24gKGV2ZW50TmFtZSwgY2FsbGJhY2ssIGN0eCkge1xuICAgICAgaWYgKHR5cGVvZiBjYWxsYmFjayAhPT0gJ2Z1bmN0aW9uJykge1xuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoJ2NhbGxiYWNrIGlzIGV4cGVjdGVkIHRvIGJlIGEgZnVuY3Rpb24nKTtcbiAgICAgIH1cbiAgICAgIHZhciBoYW5kbGVycyA9IHJlZ2lzdGVyZWRFdmVudHNbZXZlbnROYW1lXTtcbiAgICAgIGlmICghaGFuZGxlcnMpIHtcbiAgICAgICAgaGFuZGxlcnMgPSByZWdpc3RlcmVkRXZlbnRzW2V2ZW50TmFtZV0gPSBbXTtcbiAgICAgIH1cbiAgICAgIGhhbmRsZXJzLnB1c2goe2NhbGxiYWNrOiBjYWxsYmFjaywgY3R4OiBjdHh9KTtcblxuICAgICAgcmV0dXJuIHN1YmplY3Q7XG4gICAgfSxcblxuICAgIG9mZjogZnVuY3Rpb24gKGV2ZW50TmFtZSwgY2FsbGJhY2spIHtcbiAgICAgIHZhciB3YW50VG9SZW1vdmVBbGwgPSAodHlwZW9mIGV2ZW50TmFtZSA9PT0gJ3VuZGVmaW5lZCcpO1xuICAgICAgaWYgKHdhbnRUb1JlbW92ZUFsbCkge1xuICAgICAgICAvLyBLaWxsaW5nIG9sZCBldmVudHMgc3RvcmFnZSBzaG91bGQgYmUgZW5vdWdoIGluIHRoaXMgY2FzZTpcbiAgICAgICAgcmVnaXN0ZXJlZEV2ZW50cyA9IE9iamVjdC5jcmVhdGUobnVsbCk7XG4gICAgICAgIHJldHVybiBzdWJqZWN0O1xuICAgICAgfVxuXG4gICAgICBpZiAocmVnaXN0ZXJlZEV2ZW50c1tldmVudE5hbWVdKSB7XG4gICAgICAgIHZhciBkZWxldGVBbGxDYWxsYmFja3NGb3JFdmVudCA9ICh0eXBlb2YgY2FsbGJhY2sgIT09ICdmdW5jdGlvbicpO1xuICAgICAgICBpZiAoZGVsZXRlQWxsQ2FsbGJhY2tzRm9yRXZlbnQpIHtcbiAgICAgICAgICBkZWxldGUgcmVnaXN0ZXJlZEV2ZW50c1tldmVudE5hbWVdO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIHZhciBjYWxsYmFja3MgPSByZWdpc3RlcmVkRXZlbnRzW2V2ZW50TmFtZV07XG4gICAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBjYWxsYmFja3MubGVuZ3RoOyArK2kpIHtcbiAgICAgICAgICAgIGlmIChjYWxsYmFja3NbaV0uY2FsbGJhY2sgPT09IGNhbGxiYWNrKSB7XG4gICAgICAgICAgICAgIGNhbGxiYWNrcy5zcGxpY2UoaSwgMSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIHJldHVybiBzdWJqZWN0O1xuICAgIH0sXG5cbiAgICBmaXJlOiBmdW5jdGlvbiAoZXZlbnROYW1lKSB7XG4gICAgICB2YXIgY2FsbGJhY2tzID0gcmVnaXN0ZXJlZEV2ZW50c1tldmVudE5hbWVdO1xuICAgICAgaWYgKCFjYWxsYmFja3MpIHtcbiAgICAgICAgcmV0dXJuIHN1YmplY3Q7XG4gICAgICB9XG5cbiAgICAgIHZhciBmaXJlQXJndW1lbnRzO1xuICAgICAgaWYgKGFyZ3VtZW50cy5sZW5ndGggPiAxKSB7XG4gICAgICAgIGZpcmVBcmd1bWVudHMgPSBBcnJheS5wcm90b3R5cGUuc3BsaWNlLmNhbGwoYXJndW1lbnRzLCAxKTtcbiAgICAgIH1cbiAgICAgIGZvcih2YXIgaSA9IDA7IGkgPCBjYWxsYmFja3MubGVuZ3RoOyArK2kpIHtcbiAgICAgICAgdmFyIGNhbGxiYWNrSW5mbyA9IGNhbGxiYWNrc1tpXTtcbiAgICAgICAgY2FsbGJhY2tJbmZvLmNhbGxiYWNrLmFwcGx5KGNhbGxiYWNrSW5mby5jdHgsIGZpcmVBcmd1bWVudHMpO1xuICAgICAgfVxuXG4gICAgICByZXR1cm4gc3ViamVjdDtcbiAgICB9XG4gIH07XG59XG5cbmZ1bmN0aW9uIHZhbGlkYXRlU3ViamVjdChzdWJqZWN0KSB7XG4gIGlmICghc3ViamVjdCkge1xuICAgIHRocm93IG5ldyBFcnJvcignRXZlbnRpZnkgY2Fubm90IHVzZSBmYWxzeSBvYmplY3QgYXMgZXZlbnRzIHN1YmplY3QnKTtcbiAgfVxuICB2YXIgcmVzZXJ2ZWRXb3JkcyA9IFsnb24nLCAnZmlyZScsICdvZmYnXTtcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCByZXNlcnZlZFdvcmRzLmxlbmd0aDsgKytpKSB7XG4gICAgaWYgKHN1YmplY3QuaGFzT3duUHJvcGVydHkocmVzZXJ2ZWRXb3Jkc1tpXSkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihcIlN1YmplY3QgY2Fubm90IGJlIGV2ZW50aWZpZWQsIHNpbmNlIGl0IGFscmVhZHkgaGFzIHByb3BlcnR5ICdcIiArIHJlc2VydmVkV29yZHNbaV0gKyBcIidcIik7XG4gICAgfVxuICB9XG59XG4iLCJtb2R1bGUuZXhwb3J0cyA9IGV4cG9zZVByb3BlcnRpZXM7XG5cbi8qKlxuICogQXVnbWVudHMgYHRhcmdldGAgb2JqZWN0IHdpdGggZ2V0dGVyL3NldHRlciBmdW5jdGlvbnMsIHdoaWNoIG1vZGlmeSBzZXR0aW5nc1xuICpcbiAqIEBleGFtcGxlXG4gKiAgdmFyIHRhcmdldCA9IHt9O1xuICogIGV4cG9zZVByb3BlcnRpZXMoeyBhZ2U6IDQyfSwgdGFyZ2V0KTtcbiAqICB0YXJnZXQuYWdlKCk7IC8vIHJldHVybnMgNDJcbiAqICB0YXJnZXQuYWdlKDI0KTsgLy8gbWFrZSBhZ2UgMjQ7XG4gKlxuICogIHZhciBmaWx0ZXJlZFRhcmdldCA9IHt9O1xuICogIGV4cG9zZVByb3BlcnRpZXMoeyBhZ2U6IDQyLCBuYW1lOiAnSm9obid9LCBmaWx0ZXJlZFRhcmdldCwgWyduYW1lJ10pO1xuICogIGZpbHRlcmVkVGFyZ2V0Lm5hbWUoKTsgLy8gcmV0dXJucyAnSm9obidcbiAqICBmaWx0ZXJlZFRhcmdldC5hZ2UgPT09IHVuZGVmaW5lZDsgLy8gdHJ1ZVxuICovXG5mdW5jdGlvbiBleHBvc2VQcm9wZXJ0aWVzKHNldHRpbmdzLCB0YXJnZXQsIGZpbHRlcikge1xuICB2YXIgbmVlZHNGaWx0ZXIgPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwoZmlsdGVyKSA9PT0gJ1tvYmplY3QgQXJyYXldJztcbiAgaWYgKG5lZWRzRmlsdGVyKSB7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBmaWx0ZXIubGVuZ3RoOyArK2kpIHtcbiAgICAgIGF1Z21lbnQoc2V0dGluZ3MsIHRhcmdldCwgZmlsdGVyW2ldKTtcbiAgICB9XG4gIH0gZWxzZSB7XG4gICAgZm9yICh2YXIga2V5IGluIHNldHRpbmdzKSB7XG4gICAgICBhdWdtZW50KHNldHRpbmdzLCB0YXJnZXQsIGtleSk7XG4gICAgfVxuICB9XG59XG5cbmZ1bmN0aW9uIGF1Z21lbnQoc291cmNlLCB0YXJnZXQsIGtleSkge1xuICBpZiAoc291cmNlLmhhc093blByb3BlcnR5KGtleSkpIHtcbiAgICBpZiAodHlwZW9mIHRhcmdldFtrZXldID09PSAnZnVuY3Rpb24nKSB7XG4gICAgICAvLyB0aGlzIGFjY2Vzc29yIGlzIGFscmVhZHkgZGVmaW5lZC4gSWdub3JlIGl0XG4gICAgICByZXR1cm47XG4gICAgfVxuICAgIHRhcmdldFtrZXldID0gZnVuY3Rpb24gKHZhbHVlKSB7XG4gICAgICBpZiAodmFsdWUgIT09IHVuZGVmaW5lZCkge1xuICAgICAgICBzb3VyY2Vba2V5XSA9IHZhbHVlO1xuICAgICAgICByZXR1cm4gdGFyZ2V0O1xuICAgICAgfVxuICAgICAgcmV0dXJuIHNvdXJjZVtrZXldO1xuICAgIH1cbiAgfVxufVxuIiwibW9kdWxlLmV4cG9ydHMgPSBjcmVhdGVMYXlvdXQ7XG5tb2R1bGUuZXhwb3J0cy5zaW11bGF0b3IgPSByZXF1aXJlKCduZ3JhcGgucGh5c2ljcy5zaW11bGF0b3InKTtcblxudmFyIGV2ZW50aWZ5ID0gcmVxdWlyZSgnbmdyYXBoLmV2ZW50cycpO1xuXG4vKipcbiAqIENyZWF0ZXMgZm9yY2UgYmFzZWQgbGF5b3V0IGZvciBhIGdpdmVuIGdyYXBoLlxuICpcbiAqIEBwYXJhbSB7bmdyYXBoLmdyYXBofSBncmFwaCB3aGljaCBuZWVkcyB0byBiZSBsYWlkIG91dFxuICogQHBhcmFtIHtvYmplY3R9IHBoeXNpY3NTZXR0aW5ncyBpZiB5b3UgbmVlZCBjdXN0b20gc2V0dGluZ3NcbiAqIGZvciBwaHlzaWNzIHNpbXVsYXRvciB5b3UgY2FuIHBhc3MgeW91ciBvd24gc2V0dGluZ3MgaGVyZS4gSWYgaXQncyBub3QgcGFzc2VkXG4gKiBhIGRlZmF1bHQgb25lIHdpbGwgYmUgY3JlYXRlZC5cbiAqL1xuZnVuY3Rpb24gY3JlYXRlTGF5b3V0KGdyYXBoLCBwaHlzaWNzU2V0dGluZ3MpIHtcbiAgaWYgKCFncmFwaCkge1xuICAgIHRocm93IG5ldyBFcnJvcignR3JhcGggc3RydWN0dXJlIGNhbm5vdCBiZSB1bmRlZmluZWQnKTtcbiAgfVxuXG4gIHZhciBjcmVhdGVTaW11bGF0b3IgPSByZXF1aXJlKCduZ3JhcGgucGh5c2ljcy5zaW11bGF0b3InKTtcbiAgdmFyIHBoeXNpY3NTaW11bGF0b3IgPSBjcmVhdGVTaW11bGF0b3IocGh5c2ljc1NldHRpbmdzKTtcblxuICB2YXIgbm9kZUJvZGllcyA9IE9iamVjdC5jcmVhdGUobnVsbCk7XG4gIHZhciBzcHJpbmdzID0ge307XG4gIHZhciBib2RpZXNDb3VudCA9IDA7XG5cbiAgdmFyIHNwcmluZ1RyYW5zZm9ybSA9IHBoeXNpY3NTaW11bGF0b3Iuc2V0dGluZ3Muc3ByaW5nVHJhbnNmb3JtIHx8IG5vb3A7XG5cbiAgLy8gSW5pdGlhbGl6ZSBwaHlzaWNzIHdpdGggd2hhdCB3ZSBoYXZlIGluIHRoZSBncmFwaDpcbiAgaW5pdFBoeXNpY3MoKTtcbiAgbGlzdGVuVG9FdmVudHMoKTtcblxuICB2YXIgd2FzU3RhYmxlID0gZmFsc2U7XG5cbiAgdmFyIGFwaSA9IHtcbiAgICAvKipcbiAgICAgKiBQZXJmb3JtcyBvbmUgc3RlcCBvZiBpdGVyYXRpdmUgbGF5b3V0IGFsZ29yaXRobVxuICAgICAqXG4gICAgICogQHJldHVybnMge2Jvb2xlYW59IHRydWUgaWYgdGhlIHN5c3RlbSBzaG91bGQgYmUgY29uc2lkZXJlZCBzdGFibGU7IEZsYXNlIG90aGVyd2lzZS5cbiAgICAgKiBUaGUgc3lzdGVtIGlzIHN0YWJsZSBpZiBubyBmdXJ0aGVyIGNhbGwgdG8gYHN0ZXAoKWAgY2FuIGltcHJvdmUgdGhlIGxheW91dC5cbiAgICAgKi9cbiAgICBzdGVwOiBmdW5jdGlvbigpIHtcbiAgICAgIGlmIChib2RpZXNDb3VudCA9PT0gMCkgcmV0dXJuIHRydWU7IC8vIFRPRE86IFRoaXMgd2lsbCBuZXZlciBmaXJlICdzdGFibGUnXG5cbiAgICAgIHZhciBsYXN0TW92ZSA9IHBoeXNpY3NTaW11bGF0b3Iuc3RlcCgpO1xuXG4gICAgICAvLyBTYXZlIHRoZSBtb3ZlbWVudCBpbiBjYXNlIGlmIHNvbWVvbmUgd2FudHMgdG8gcXVlcnkgaXQgaW4gdGhlIHN0ZXBcbiAgICAgIC8vIGNhbGxiYWNrLlxuICAgICAgYXBpLmxhc3RNb3ZlID0gbGFzdE1vdmU7XG5cbiAgICAgIC8vIEFsbG93IGxpc3RlbmVycyB0byBwZXJmb3JtIGxvdy1sZXZlbCBhY3Rpb25zIGFmdGVyIG5vZGVzIGFyZSB1cGRhdGVkLlxuICAgICAgYXBpLmZpcmUoJ3N0ZXAnKTtcblxuICAgICAgdmFyIHJhdGlvID0gbGFzdE1vdmUvYm9kaWVzQ291bnQ7XG4gICAgICB2YXIgaXNTdGFibGVOb3cgPSByYXRpbyA8PSAwLjAxOyAvLyBUT0RPOiBUaGUgbnVtYmVyIGlzIHNvbWV3aGF0IGFyYml0cmFyeS4uLlxuXG4gICAgICBpZiAod2FzU3RhYmxlICE9PSBpc1N0YWJsZU5vdykge1xuICAgICAgICB3YXNTdGFibGUgPSBpc1N0YWJsZU5vdztcbiAgICAgICAgb25TdGFibGVDaGFuZ2VkKGlzU3RhYmxlTm93KTtcbiAgICAgIH1cblxuICAgICAgcmV0dXJuIGlzU3RhYmxlTm93O1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBGb3IgYSBnaXZlbiBgbm9kZUlkYCByZXR1cm5zIHBvc2l0aW9uXG4gICAgICovXG4gICAgZ2V0Tm9kZVBvc2l0aW9uOiBmdW5jdGlvbiAobm9kZUlkKSB7XG4gICAgICByZXR1cm4gZ2V0SW5pdGlhbGl6ZWRCb2R5KG5vZGVJZCkucG9zO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBTZXRzIHBvc2l0aW9uIG9mIGEgbm9kZSB0byBhIGdpdmVuIGNvb3JkaW5hdGVzXG4gICAgICogQHBhcmFtIHtzdHJpbmd9IG5vZGVJZCBub2RlIGlkZW50aWZpZXJcbiAgICAgKiBAcGFyYW0ge251bWJlcn0geCBwb3NpdGlvbiBvZiBhIG5vZGVcbiAgICAgKiBAcGFyYW0ge251bWJlcn0geSBwb3NpdGlvbiBvZiBhIG5vZGVcbiAgICAgKiBAcGFyYW0ge251bWJlcj19IHogcG9zaXRpb24gb2Ygbm9kZSAob25seSBpZiBhcHBsaWNhYmxlIHRvIGJvZHkpXG4gICAgICovXG4gICAgc2V0Tm9kZVBvc2l0aW9uOiBmdW5jdGlvbiAobm9kZUlkKSB7XG4gICAgICB2YXIgYm9keSA9IGdldEluaXRpYWxpemVkQm9keShub2RlSWQpO1xuICAgICAgYm9keS5zZXRQb3NpdGlvbi5hcHBseShib2R5LCBBcnJheS5wcm90b3R5cGUuc2xpY2UuY2FsbChhcmd1bWVudHMsIDEpKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQHJldHVybnMge09iamVjdH0gTGluayBwb3NpdGlvbiBieSBsaW5rIGlkXG4gICAgICogQHJldHVybnMge09iamVjdC5mcm9tfSB7eCwgeX0gY29vcmRpbmF0ZXMgb2YgbGluayBzdGFydFxuICAgICAqIEByZXR1cm5zIHtPYmplY3QudG99IHt4LCB5fSBjb29yZGluYXRlcyBvZiBsaW5rIGVuZFxuICAgICAqL1xuICAgIGdldExpbmtQb3NpdGlvbjogZnVuY3Rpb24gKGxpbmtJZCkge1xuICAgICAgdmFyIHNwcmluZyA9IHNwcmluZ3NbbGlua0lkXTtcbiAgICAgIGlmIChzcHJpbmcpIHtcbiAgICAgICAgcmV0dXJuIHtcbiAgICAgICAgICBmcm9tOiBzcHJpbmcuZnJvbS5wb3MsXG4gICAgICAgICAgdG86IHNwcmluZy50by5wb3NcbiAgICAgICAgfTtcbiAgICAgIH1cbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQHJldHVybnMge09iamVjdH0gYXJlYSByZXF1aXJlZCB0byBmaXQgaW4gdGhlIGdyYXBoLiBPYmplY3QgY29udGFpbnNcbiAgICAgKiBgeDFgLCBgeTFgIC0gdG9wIGxlZnQgY29vcmRpbmF0ZXNcbiAgICAgKiBgeDJgLCBgeTJgIC0gYm90dG9tIHJpZ2h0IGNvb3JkaW5hdGVzXG4gICAgICovXG4gICAgZ2V0R3JhcGhSZWN0OiBmdW5jdGlvbiAoKSB7XG4gICAgICByZXR1cm4gcGh5c2ljc1NpbXVsYXRvci5nZXRCQm94KCk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEl0ZXJhdGVzIG92ZXIgZWFjaCBib2R5IGluIHRoZSBsYXlvdXQgc2ltdWxhdG9yIGFuZCBwZXJmb3JtcyBhIGNhbGxiYWNrKGJvZHksIG5vZGVJZClcbiAgICAgKi9cbiAgICBmb3JFYWNoQm9keTogZm9yRWFjaEJvZHksXG5cbiAgICAvKlxuICAgICAqIFJlcXVlc3RzIGxheW91dCBhbGdvcml0aG0gdG8gcGluL3VucGluIG5vZGUgdG8gaXRzIGN1cnJlbnQgcG9zaXRpb25cbiAgICAgKiBQaW5uZWQgbm9kZXMgc2hvdWxkIG5vdCBiZSBhZmZlY3RlZCBieSBsYXlvdXQgYWxnb3JpdGhtIGFuZCBhbHdheXNcbiAgICAgKiByZW1haW4gYXQgdGhlaXIgcG9zaXRpb25cbiAgICAgKi9cbiAgICBwaW5Ob2RlOiBmdW5jdGlvbiAobm9kZSwgaXNQaW5uZWQpIHtcbiAgICAgIHZhciBib2R5ID0gZ2V0SW5pdGlhbGl6ZWRCb2R5KG5vZGUuaWQpO1xuICAgICAgIGJvZHkuaXNQaW5uZWQgPSAhIWlzUGlubmVkO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3Mgd2hldGhlciBnaXZlbiBncmFwaCdzIG5vZGUgaXMgY3VycmVudGx5IHBpbm5lZFxuICAgICAqL1xuICAgIGlzTm9kZVBpbm5lZDogZnVuY3Rpb24gKG5vZGUpIHtcbiAgICAgIHJldHVybiBnZXRJbml0aWFsaXplZEJvZHkobm9kZS5pZCkuaXNQaW5uZWQ7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIFJlcXVlc3QgdG8gcmVsZWFzZSBhbGwgcmVzb3VyY2VzXG4gICAgICovXG4gICAgZGlzcG9zZTogZnVuY3Rpb24oKSB7XG4gICAgICBncmFwaC5vZmYoJ2NoYW5nZWQnLCBvbkdyYXBoQ2hhbmdlZCk7XG4gICAgICBhcGkuZmlyZSgnZGlzcG9zZWQnKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogR2V0cyBwaHlzaWNhbCBib2R5IGZvciBhIGdpdmVuIG5vZGUgaWQuIElmIG5vZGUgaXMgbm90IGZvdW5kIHVuZGVmaW5lZFxuICAgICAqIHZhbHVlIGlzIHJldHVybmVkLlxuICAgICAqL1xuICAgIGdldEJvZHk6IGdldEJvZHksXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHNwcmluZyBmb3IgYSBnaXZlbiBlZGdlLlxuICAgICAqXG4gICAgICogQHBhcmFtIHtzdHJpbmd9IGxpbmtJZCBsaW5rIGlkZW50aWZlci4gSWYgdHdvIGFyZ3VtZW50cyBhcmUgcGFzc2VkIHRoZW5cbiAgICAgKiB0aGlzIGFyZ3VtZW50IGlzIHRyZWF0ZWQgYXMgZm9ybU5vZGVJZFxuICAgICAqIEBwYXJhbSB7c3RyaW5nPX0gdG9JZCB3aGVuIGRlZmluZWQgdGhpcyBwYXJhbWV0ZXIgZGVub3RlcyBoZWFkIG9mIHRoZSBsaW5rXG4gICAgICogYW5kIGZpcnN0IGFyZ3VtZW50IGlzIHRyYXRlZCBhcyB0YWlsIG9mIHRoZSBsaW5rIChmcm9tSWQpXG4gICAgICovXG4gICAgZ2V0U3ByaW5nOiBnZXRTcHJpbmcsXG5cbiAgICAvKipcbiAgICAgKiBbUmVhZCBvbmx5XSBHZXRzIGN1cnJlbnQgcGh5c2ljcyBzaW11bGF0b3JcbiAgICAgKi9cbiAgICBzaW11bGF0b3I6IHBoeXNpY3NTaW11bGF0b3IsXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBncmFwaCB0aGF0IHdhcyB1c2VkIGZvciBsYXlvdXRcbiAgICAgKi9cbiAgICBncmFwaDogZ3JhcGgsXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIGFtb3VudCBvZiBtb3ZlbWVudCBwZXJmb3JtZWQgZHVyaW5nIGxhc3Qgc3RlcCBvcGVhcnRpb25cbiAgICAgKi9cbiAgICBsYXN0TW92ZTogMFxuICB9O1xuXG4gIGV2ZW50aWZ5KGFwaSk7XG5cbiAgcmV0dXJuIGFwaTtcblxuICBmdW5jdGlvbiBmb3JFYWNoQm9keShjYikge1xuICAgIE9iamVjdC5rZXlzKG5vZGVCb2RpZXMpLmZvckVhY2goZnVuY3Rpb24oYm9keUlkKSB7XG4gICAgICBjYihub2RlQm9kaWVzW2JvZHlJZF0sIGJvZHlJZCk7XG4gICAgfSk7XG4gIH1cblxuICBmdW5jdGlvbiBnZXRTcHJpbmcoZnJvbUlkLCB0b0lkKSB7XG4gICAgdmFyIGxpbmtJZDtcbiAgICBpZiAodG9JZCA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICBpZiAodHlwZW9mIGZyb21JZCAhPT0gJ29iamVjdCcpIHtcbiAgICAgICAgLy8gYXNzdW1lIGZyb21JZCBhcyBhIGxpbmtJZDpcbiAgICAgICAgbGlua0lkID0gZnJvbUlkO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgLy8gYXNzdW1lIGZyb21JZCB0byBiZSBhIGxpbmsgb2JqZWN0OlxuICAgICAgICBsaW5rSWQgPSBmcm9tSWQuaWQ7XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIC8vIHRvSWQgaXMgZGVmaW5lZCwgc2hvdWxkIGdyYWIgbGluazpcbiAgICAgIHZhciBsaW5rID0gZ3JhcGguaGFzTGluayhmcm9tSWQsIHRvSWQpO1xuICAgICAgaWYgKCFsaW5rKSByZXR1cm47XG4gICAgICBsaW5rSWQgPSBsaW5rLmlkO1xuICAgIH1cblxuICAgIHJldHVybiBzcHJpbmdzW2xpbmtJZF07XG4gIH1cblxuICBmdW5jdGlvbiBnZXRCb2R5KG5vZGVJZCkge1xuICAgIHJldHVybiBub2RlQm9kaWVzW25vZGVJZF07XG4gIH1cblxuICBmdW5jdGlvbiBsaXN0ZW5Ub0V2ZW50cygpIHtcbiAgICBncmFwaC5vbignY2hhbmdlZCcsIG9uR3JhcGhDaGFuZ2VkKTtcbiAgfVxuXG4gIGZ1bmN0aW9uIG9uU3RhYmxlQ2hhbmdlZChpc1N0YWJsZSkge1xuICAgIGFwaS5maXJlKCdzdGFibGUnLCBpc1N0YWJsZSk7XG4gIH1cblxuICBmdW5jdGlvbiBvbkdyYXBoQ2hhbmdlZChjaGFuZ2VzKSB7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBjaGFuZ2VzLmxlbmd0aDsgKytpKSB7XG4gICAgICB2YXIgY2hhbmdlID0gY2hhbmdlc1tpXTtcbiAgICAgIGlmIChjaGFuZ2UuY2hhbmdlVHlwZSA9PT0gJ2FkZCcpIHtcbiAgICAgICAgaWYgKGNoYW5nZS5ub2RlKSB7XG4gICAgICAgICAgaW5pdEJvZHkoY2hhbmdlLm5vZGUuaWQpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChjaGFuZ2UubGluaykge1xuICAgICAgICAgIGluaXRMaW5rKGNoYW5nZS5saW5rKTtcbiAgICAgICAgfVxuICAgICAgfSBlbHNlIGlmIChjaGFuZ2UuY2hhbmdlVHlwZSA9PT0gJ3JlbW92ZScpIHtcbiAgICAgICAgaWYgKGNoYW5nZS5ub2RlKSB7XG4gICAgICAgICAgcmVsZWFzZU5vZGUoY2hhbmdlLm5vZGUpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChjaGFuZ2UubGluaykge1xuICAgICAgICAgIHJlbGVhc2VMaW5rKGNoYW5nZS5saW5rKTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgICBib2RpZXNDb3VudCA9IGdyYXBoLmdldE5vZGVzQ291bnQoKTtcbiAgfVxuXG4gIGZ1bmN0aW9uIGluaXRQaHlzaWNzKCkge1xuICAgIGJvZGllc0NvdW50ID0gMDtcblxuICAgIGdyYXBoLmZvckVhY2hOb2RlKGZ1bmN0aW9uIChub2RlKSB7XG4gICAgICBpbml0Qm9keShub2RlLmlkKTtcbiAgICAgIGJvZGllc0NvdW50ICs9IDE7XG4gICAgfSk7XG5cbiAgICBncmFwaC5mb3JFYWNoTGluayhpbml0TGluayk7XG4gIH1cblxuICBmdW5jdGlvbiBpbml0Qm9keShub2RlSWQpIHtcbiAgICB2YXIgYm9keSA9IG5vZGVCb2RpZXNbbm9kZUlkXTtcbiAgICBpZiAoIWJvZHkpIHtcbiAgICAgIHZhciBub2RlID0gZ3JhcGguZ2V0Tm9kZShub2RlSWQpO1xuICAgICAgaWYgKCFub2RlKSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcignaW5pdEJvZHkoKSB3YXMgY2FsbGVkIHdpdGggdW5rbm93biBub2RlIGlkJyk7XG4gICAgICB9XG5cbiAgICAgIHZhciBwb3MgPSBub2RlLnBvc2l0aW9uO1xuICAgICAgaWYgKCFwb3MpIHtcbiAgICAgICAgdmFyIG5laWdoYm9ycyA9IGdldE5laWdoYm9yQm9kaWVzKG5vZGUpO1xuICAgICAgICBwb3MgPSBwaHlzaWNzU2ltdWxhdG9yLmdldEJlc3ROZXdCb2R5UG9zaXRpb24obmVpZ2hib3JzKTtcbiAgICAgIH1cblxuICAgICAgYm9keSA9IHBoeXNpY3NTaW11bGF0b3IuYWRkQm9keUF0KHBvcyk7XG4gICAgICBib2R5LmlkID0gbm9kZUlkO1xuXG4gICAgICBub2RlQm9kaWVzW25vZGVJZF0gPSBib2R5O1xuICAgICAgdXBkYXRlQm9keU1hc3Mobm9kZUlkKTtcblxuICAgICAgaWYgKGlzTm9kZU9yaWdpbmFsbHlQaW5uZWQobm9kZSkpIHtcbiAgICAgICAgYm9keS5pc1Bpbm5lZCA9IHRydWU7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgZnVuY3Rpb24gcmVsZWFzZU5vZGUobm9kZSkge1xuICAgIHZhciBub2RlSWQgPSBub2RlLmlkO1xuICAgIHZhciBib2R5ID0gbm9kZUJvZGllc1tub2RlSWRdO1xuICAgIGlmIChib2R5KSB7XG4gICAgICBub2RlQm9kaWVzW25vZGVJZF0gPSBudWxsO1xuICAgICAgZGVsZXRlIG5vZGVCb2RpZXNbbm9kZUlkXTtcblxuICAgICAgcGh5c2ljc1NpbXVsYXRvci5yZW1vdmVCb2R5KGJvZHkpO1xuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIGluaXRMaW5rKGxpbmspIHtcbiAgICB1cGRhdGVCb2R5TWFzcyhsaW5rLmZyb21JZCk7XG4gICAgdXBkYXRlQm9keU1hc3MobGluay50b0lkKTtcblxuICAgIHZhciBmcm9tQm9keSA9IG5vZGVCb2RpZXNbbGluay5mcm9tSWRdLFxuICAgICAgICB0b0JvZHkgID0gbm9kZUJvZGllc1tsaW5rLnRvSWRdLFxuICAgICAgICBzcHJpbmcgPSBwaHlzaWNzU2ltdWxhdG9yLmFkZFNwcmluZyhmcm9tQm9keSwgdG9Cb2R5LCBsaW5rLmxlbmd0aCk7XG5cbiAgICBzcHJpbmdUcmFuc2Zvcm0obGluaywgc3ByaW5nKTtcblxuICAgIHNwcmluZ3NbbGluay5pZF0gPSBzcHJpbmc7XG4gIH1cblxuICBmdW5jdGlvbiByZWxlYXNlTGluayhsaW5rKSB7XG4gICAgdmFyIHNwcmluZyA9IHNwcmluZ3NbbGluay5pZF07XG4gICAgaWYgKHNwcmluZykge1xuICAgICAgdmFyIGZyb20gPSBncmFwaC5nZXROb2RlKGxpbmsuZnJvbUlkKSxcbiAgICAgICAgICB0byA9IGdyYXBoLmdldE5vZGUobGluay50b0lkKTtcblxuICAgICAgaWYgKGZyb20pIHVwZGF0ZUJvZHlNYXNzKGZyb20uaWQpO1xuICAgICAgaWYgKHRvKSB1cGRhdGVCb2R5TWFzcyh0by5pZCk7XG5cbiAgICAgIGRlbGV0ZSBzcHJpbmdzW2xpbmsuaWRdO1xuXG4gICAgICBwaHlzaWNzU2ltdWxhdG9yLnJlbW92ZVNwcmluZyhzcHJpbmcpO1xuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIGdldE5laWdoYm9yQm9kaWVzKG5vZGUpIHtcbiAgICAvLyBUT0RPOiBDb3VsZCBwcm9iYWJseSBiZSBkb25lIGJldHRlciBvbiBtZW1vcnlcbiAgICB2YXIgbmVpZ2hib3JzID0gW107XG4gICAgaWYgKCFub2RlLmxpbmtzKSB7XG4gICAgICByZXR1cm4gbmVpZ2hib3JzO1xuICAgIH1cbiAgICB2YXIgbWF4TmVpZ2hib3JzID0gTWF0aC5taW4obm9kZS5saW5rcy5sZW5ndGgsIDIpO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbWF4TmVpZ2hib3JzOyArK2kpIHtcbiAgICAgIHZhciBsaW5rID0gbm9kZS5saW5rc1tpXTtcbiAgICAgIHZhciBvdGhlckJvZHkgPSBsaW5rLmZyb21JZCAhPT0gbm9kZS5pZCA/IG5vZGVCb2RpZXNbbGluay5mcm9tSWRdIDogbm9kZUJvZGllc1tsaW5rLnRvSWRdO1xuICAgICAgaWYgKG90aGVyQm9keSAmJiBvdGhlckJvZHkucG9zKSB7XG4gICAgICAgIG5laWdoYm9ycy5wdXNoKG90aGVyQm9keSk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgcmV0dXJuIG5laWdoYm9ycztcbiAgfVxuXG4gIGZ1bmN0aW9uIHVwZGF0ZUJvZHlNYXNzKG5vZGVJZCkge1xuICAgIHZhciBib2R5ID0gbm9kZUJvZGllc1tub2RlSWRdO1xuICAgIGJvZHkubWFzcyA9IG5vZGVNYXNzKG5vZGVJZCk7XG4gIH1cblxuICAvKipcbiAgICogQ2hlY2tzIHdoZXRoZXIgZ3JhcGggbm9kZSBoYXMgaW4gaXRzIHNldHRpbmdzIHBpbm5lZCBhdHRyaWJ1dGUsXG4gICAqIHdoaWNoIG1lYW5zIGxheW91dCBhbGdvcml0aG0gY2Fubm90IG1vdmUgaXQuIE5vZGUgY2FuIGJlIHByZWNvbmZpZ3VyZWRcbiAgICogYXMgcGlubmVkLCBpZiBpdCBoYXMgXCJpc1Bpbm5lZFwiIGF0dHJpYnV0ZSwgb3Igd2hlbiBub2RlLmRhdGEgaGFzIGl0LlxuICAgKlxuICAgKiBAcGFyYW0ge09iamVjdH0gbm9kZSBhIGdyYXBoIG5vZGUgdG8gY2hlY2tcbiAgICogQHJldHVybiB7Qm9vbGVhbn0gdHJ1ZSBpZiBub2RlIHNob3VsZCBiZSB0cmVhdGVkIGFzIHBpbm5lZDsgZmFsc2Ugb3RoZXJ3aXNlLlxuICAgKi9cbiAgZnVuY3Rpb24gaXNOb2RlT3JpZ2luYWxseVBpbm5lZChub2RlKSB7XG4gICAgcmV0dXJuIChub2RlICYmIChub2RlLmlzUGlubmVkIHx8IChub2RlLmRhdGEgJiYgbm9kZS5kYXRhLmlzUGlubmVkKSkpO1xuICB9XG5cbiAgZnVuY3Rpb24gZ2V0SW5pdGlhbGl6ZWRCb2R5KG5vZGVJZCkge1xuICAgIHZhciBib2R5ID0gbm9kZUJvZGllc1tub2RlSWRdO1xuICAgIGlmICghYm9keSkge1xuICAgICAgaW5pdEJvZHkobm9kZUlkKTtcbiAgICAgIGJvZHkgPSBub2RlQm9kaWVzW25vZGVJZF07XG4gICAgfVxuICAgIHJldHVybiBib2R5O1xuICB9XG5cbiAgLyoqXG4gICAqIENhbGN1bGF0ZXMgbWFzcyBvZiBhIGJvZHksIHdoaWNoIGNvcnJlc3BvbmRzIHRvIG5vZGUgd2l0aCBnaXZlbiBpZC5cbiAgICpcbiAgICogQHBhcmFtIHtTdHJpbmd8TnVtYmVyfSBub2RlSWQgaWRlbnRpZmllciBvZiBhIG5vZGUsIGZvciB3aGljaCBib2R5IG1hc3MgbmVlZHMgdG8gYmUgY2FsY3VsYXRlZFxuICAgKiBAcmV0dXJucyB7TnVtYmVyfSByZWNvbW1lbmRlZCBtYXNzIG9mIHRoZSBib2R5O1xuICAgKi9cbiAgZnVuY3Rpb24gbm9kZU1hc3Mobm9kZUlkKSB7XG4gICAgdmFyIGxpbmtzID0gZ3JhcGguZ2V0TGlua3Mobm9kZUlkKTtcbiAgICBpZiAoIWxpbmtzKSByZXR1cm4gMTtcbiAgICByZXR1cm4gMSArIGxpbmtzLmxlbmd0aCAvIDMuMDtcbiAgfVxufVxuXG5mdW5jdGlvbiBub29wKCkgeyB9XG4iLCIvKipcbiAqIE1hbmFnZXMgYSBzaW11bGF0aW9uIG9mIHBoeXNpY2FsIGZvcmNlcyBhY3Rpbmcgb24gYm9kaWVzIGFuZCBzcHJpbmdzLlxuICovXG5tb2R1bGUuZXhwb3J0cyA9IHBoeXNpY3NTaW11bGF0b3I7XG5cbmZ1bmN0aW9uIHBoeXNpY3NTaW11bGF0b3Ioc2V0dGluZ3MpIHtcbiAgdmFyIFNwcmluZyA9IHJlcXVpcmUoJy4vbGliL3NwcmluZycpO1xuICB2YXIgZXhwb3NlID0gcmVxdWlyZSgnbmdyYXBoLmV4cG9zZScpO1xuICB2YXIgbWVyZ2UgPSByZXF1aXJlKCduZ3JhcGgubWVyZ2UnKTtcbiAgdmFyIGV2ZW50aWZ5ID0gcmVxdWlyZSgnbmdyYXBoLmV2ZW50cycpO1xuXG4gIHNldHRpbmdzID0gbWVyZ2Uoc2V0dGluZ3MsIHtcbiAgICAgIC8qKlxuICAgICAgICogSWRlYWwgbGVuZ3RoIGZvciBsaW5rcyAoc3ByaW5ncyBpbiBwaHlzaWNhbCBtb2RlbCkuXG4gICAgICAgKi9cbiAgICAgIHNwcmluZ0xlbmd0aDogMzAsXG5cbiAgICAgIC8qKlxuICAgICAgICogSG9vaydzIGxhdyBjb2VmZmljaWVudC4gMSAtIHNvbGlkIHNwcmluZy5cbiAgICAgICAqL1xuICAgICAgc3ByaW5nQ29lZmY6IDAuMDAwOCxcblxuICAgICAgLyoqXG4gICAgICAgKiBDb3Vsb21iJ3MgbGF3IGNvZWZmaWNpZW50LiBJdCdzIHVzZWQgdG8gcmVwZWwgbm9kZXMgdGh1cyBzaG91bGQgYmUgbmVnYXRpdmVcbiAgICAgICAqIGlmIHlvdSBtYWtlIGl0IHBvc2l0aXZlIG5vZGVzIHN0YXJ0IGF0dHJhY3QgZWFjaCBvdGhlciA6KS5cbiAgICAgICAqL1xuICAgICAgZ3Jhdml0eTogLTEuMixcblxuICAgICAgLyoqXG4gICAgICAgKiBUaGV0YSBjb2VmZmljaWVudCBmcm9tIEJhcm5lcyBIdXQgc2ltdWxhdGlvbi4gUmFuZ2VkIGJldHdlZW4gKDAsIDEpLlxuICAgICAgICogVGhlIGNsb3NlciBpdCdzIHRvIDEgdGhlIG1vcmUgbm9kZXMgYWxnb3JpdGhtIHdpbGwgaGF2ZSB0byBnbyB0aHJvdWdoLlxuICAgICAgICogU2V0dGluZyBpdCB0byBvbmUgbWFrZXMgQmFybmVzIEh1dCBzaW11bGF0aW9uIG5vIGRpZmZlcmVudCBmcm9tXG4gICAgICAgKiBicnV0ZS1mb3JjZSBmb3JjZXMgY2FsY3VsYXRpb24gKGVhY2ggbm9kZSBpcyBjb25zaWRlcmVkKS5cbiAgICAgICAqL1xuICAgICAgdGhldGE6IDAuOCxcblxuICAgICAgLyoqXG4gICAgICAgKiBEcmFnIGZvcmNlIGNvZWZmaWNpZW50LiBVc2VkIHRvIHNsb3cgZG93biBzeXN0ZW0sIHRodXMgc2hvdWxkIGJlIGxlc3MgdGhhbiAxLlxuICAgICAgICogVGhlIGNsb3NlciBpdCBpcyB0byAwIHRoZSBsZXNzIHRpZ2h0IHN5c3RlbSB3aWxsIGJlLlxuICAgICAgICovXG4gICAgICBkcmFnQ29lZmY6IDAuMDIsXG5cbiAgICAgIC8qKlxuICAgICAgICogRGVmYXVsdCB0aW1lIHN0ZXAgKGR0KSBmb3IgZm9yY2VzIGludGVncmF0aW9uXG4gICAgICAgKi9cbiAgICAgIHRpbWVTdGVwIDogMjAsXG4gIH0pO1xuXG4gIC8vIFdlIGFsbG93IGNsaWVudHMgdG8gb3ZlcnJpZGUgYmFzaWMgZmFjdG9yeSBtZXRob2RzOlxuICB2YXIgY3JlYXRlUXVhZFRyZWUgPSBzZXR0aW5ncy5jcmVhdGVRdWFkVHJlZSB8fCByZXF1aXJlKCduZ3JhcGgucXVhZHRyZWViaCcpO1xuICB2YXIgY3JlYXRlQm91bmRzID0gc2V0dGluZ3MuY3JlYXRlQm91bmRzIHx8IHJlcXVpcmUoJy4vbGliL2JvdW5kcycpO1xuICB2YXIgY3JlYXRlRHJhZ0ZvcmNlID0gc2V0dGluZ3MuY3JlYXRlRHJhZ0ZvcmNlIHx8IHJlcXVpcmUoJy4vbGliL2RyYWdGb3JjZScpO1xuICB2YXIgY3JlYXRlU3ByaW5nRm9yY2UgPSBzZXR0aW5ncy5jcmVhdGVTcHJpbmdGb3JjZSB8fCByZXF1aXJlKCcuL2xpYi9zcHJpbmdGb3JjZScpO1xuICB2YXIgaW50ZWdyYXRlID0gc2V0dGluZ3MuaW50ZWdyYXRvciB8fCByZXF1aXJlKCcuL2xpYi9ldWxlckludGVncmF0b3InKTtcbiAgdmFyIGNyZWF0ZUJvZHkgPSBzZXR0aW5ncy5jcmVhdGVCb2R5IHx8IHJlcXVpcmUoJy4vbGliL2NyZWF0ZUJvZHknKTtcblxuICB2YXIgYm9kaWVzID0gW10sIC8vIEJvZGllcyBpbiB0aGlzIHNpbXVsYXRpb24uXG4gICAgICBzcHJpbmdzID0gW10sIC8vIFNwcmluZ3MgaW4gdGhpcyBzaW11bGF0aW9uLlxuICAgICAgcXVhZFRyZWUgPSAgY3JlYXRlUXVhZFRyZWUoc2V0dGluZ3MpLFxuICAgICAgYm91bmRzID0gY3JlYXRlQm91bmRzKGJvZGllcywgc2V0dGluZ3MpLFxuICAgICAgc3ByaW5nRm9yY2UgPSBjcmVhdGVTcHJpbmdGb3JjZShzZXR0aW5ncyksXG4gICAgICBkcmFnRm9yY2UgPSBjcmVhdGVEcmFnRm9yY2Uoc2V0dGluZ3MpO1xuXG4gIHZhciB0b3RhbE1vdmVtZW50ID0gMDsgLy8gaG93IG11Y2ggbW92ZW1lbnQgd2UgbWFkZSBvbiBsYXN0IHN0ZXBcblxuICB2YXIgcHVibGljQXBpID0ge1xuICAgIC8qKlxuICAgICAqIEFycmF5IG9mIGJvZGllcywgcmVnaXN0ZXJlZCB3aXRoIGN1cnJlbnQgc2ltdWxhdG9yXG4gICAgICpcbiAgICAgKiBOb3RlOiBUbyBhZGQgbmV3IGJvZHksIHVzZSBhZGRCb2R5KCkgbWV0aG9kLiBUaGlzIHByb3BlcnR5IGlzIG9ubHlcbiAgICAgKiBleHBvc2VkIGZvciB0ZXN0aW5nL3BlcmZvcm1hbmNlIHB1cnBvc2VzLlxuICAgICAqL1xuICAgIGJvZGllczogYm9kaWVzLFxuXG4gICAgcXVhZFRyZWU6IHF1YWRUcmVlLFxuXG4gICAgLyoqXG4gICAgICogQXJyYXkgb2Ygc3ByaW5ncywgcmVnaXN0ZXJlZCB3aXRoIGN1cnJlbnQgc2ltdWxhdG9yXG4gICAgICpcbiAgICAgKiBOb3RlOiBUbyBhZGQgbmV3IHNwcmluZywgdXNlIGFkZFNwcmluZygpIG1ldGhvZC4gVGhpcyBwcm9wZXJ0eSBpcyBvbmx5XG4gICAgICogZXhwb3NlZCBmb3IgdGVzdGluZy9wZXJmb3JtYW5jZSBwdXJwb3Nlcy5cbiAgICAgKi9cbiAgICBzcHJpbmdzOiBzcHJpbmdzLFxuXG4gICAgLyoqXG4gICAgICogUmV0dXJucyBzZXR0aW5ncyB3aXRoIHdoaWNoIGN1cnJlbnQgc2ltdWxhdG9yIHdhcyBpbml0aWFsaXplZFxuICAgICAqL1xuICAgIHNldHRpbmdzOiBzZXR0aW5ncyxcblxuICAgIC8qKlxuICAgICAqIFBlcmZvcm1zIG9uZSBzdGVwIG9mIGZvcmNlIHNpbXVsYXRpb24uXG4gICAgICpcbiAgICAgKiBAcmV0dXJucyB7Ym9vbGVhbn0gdHJ1ZSBpZiBzeXN0ZW0gaXMgY29uc2lkZXJlZCBzdGFibGU7IEZhbHNlIG90aGVyd2lzZS5cbiAgICAgKi9cbiAgICBzdGVwOiBmdW5jdGlvbiAoKSB7XG4gICAgICBhY2N1bXVsYXRlRm9yY2VzKCk7XG5cbiAgICAgIHZhciBtb3ZlbWVudCA9IGludGVncmF0ZShib2RpZXMsIHNldHRpbmdzLnRpbWVTdGVwKTtcbiAgICAgIGJvdW5kcy51cGRhdGUoKTtcblxuICAgICAgcmV0dXJuIG1vdmVtZW50O1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBBZGRzIGJvZHkgdG8gdGhlIHN5c3RlbVxuICAgICAqXG4gICAgICogQHBhcmFtIHtuZ3JhcGgucGh5c2ljcy5wcmltaXRpdmVzLkJvZHl9IGJvZHkgcGh5c2ljYWwgYm9keVxuICAgICAqXG4gICAgICogQHJldHVybnMge25ncmFwaC5waHlzaWNzLnByaW1pdGl2ZXMuQm9keX0gYWRkZWQgYm9keVxuICAgICAqL1xuICAgIGFkZEJvZHk6IGZ1bmN0aW9uIChib2R5KSB7XG4gICAgICBpZiAoIWJvZHkpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdCb2R5IGlzIHJlcXVpcmVkJyk7XG4gICAgICB9XG4gICAgICBib2RpZXMucHVzaChib2R5KTtcblxuICAgICAgcmV0dXJuIGJvZHk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEFkZHMgYm9keSB0byB0aGUgc3lzdGVtIGF0IGdpdmVuIHBvc2l0aW9uXG4gICAgICpcbiAgICAgKiBAcGFyYW0ge09iamVjdH0gcG9zIHBvc2l0aW9uIG9mIGEgYm9keVxuICAgICAqXG4gICAgICogQHJldHVybnMge25ncmFwaC5waHlzaWNzLnByaW1pdGl2ZXMuQm9keX0gYWRkZWQgYm9keVxuICAgICAqL1xuICAgIGFkZEJvZHlBdDogZnVuY3Rpb24gKHBvcykge1xuICAgICAgaWYgKCFwb3MpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdCb2R5IHBvc2l0aW9uIGlzIHJlcXVpcmVkJyk7XG4gICAgICB9XG4gICAgICB2YXIgYm9keSA9IGNyZWF0ZUJvZHkocG9zKTtcbiAgICAgIGJvZGllcy5wdXNoKGJvZHkpO1xuXG4gICAgICByZXR1cm4gYm9keTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogUmVtb3ZlcyBib2R5IGZyb20gdGhlIHN5c3RlbVxuICAgICAqXG4gICAgICogQHBhcmFtIHtuZ3JhcGgucGh5c2ljcy5wcmltaXRpdmVzLkJvZHl9IGJvZHkgdG8gcmVtb3ZlXG4gICAgICpcbiAgICAgKiBAcmV0dXJucyB7Qm9vbGVhbn0gdHJ1ZSBpZiBib2R5IGZvdW5kIGFuZCByZW1vdmVkLiBmYWxzeSBvdGhlcndpc2U7XG4gICAgICovXG4gICAgcmVtb3ZlQm9keTogZnVuY3Rpb24gKGJvZHkpIHtcbiAgICAgIGlmICghYm9keSkgeyByZXR1cm47IH1cblxuICAgICAgdmFyIGlkeCA9IGJvZGllcy5pbmRleE9mKGJvZHkpO1xuICAgICAgaWYgKGlkeCA8IDApIHsgcmV0dXJuOyB9XG5cbiAgICAgIGJvZGllcy5zcGxpY2UoaWR4LCAxKTtcbiAgICAgIGlmIChib2RpZXMubGVuZ3RoID09PSAwKSB7XG4gICAgICAgIGJvdW5kcy5yZXNldCgpO1xuICAgICAgfVxuICAgICAgcmV0dXJuIHRydWU7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEFkZHMgYSBzcHJpbmcgdG8gdGhpcyBzaW11bGF0aW9uLlxuICAgICAqXG4gICAgICogQHJldHVybnMge09iamVjdH0gLSBhIGhhbmRsZSBmb3IgYSBzcHJpbmcuIElmIHlvdSB3YW50IHRvIGxhdGVyIHJlbW92ZVxuICAgICAqIHNwcmluZyBwYXNzIGl0IHRvIHJlbW92ZVNwcmluZygpIG1ldGhvZC5cbiAgICAgKi9cbiAgICBhZGRTcHJpbmc6IGZ1bmN0aW9uIChib2R5MSwgYm9keTIsIHNwcmluZ0xlbmd0aCwgc3ByaW5nV2VpZ2h0LCBzcHJpbmdDb2VmZmljaWVudCkge1xuICAgICAgaWYgKCFib2R5MSB8fCAhYm9keTIpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdDYW5ub3QgYWRkIG51bGwgc3ByaW5nIHRvIGZvcmNlIHNpbXVsYXRvcicpO1xuICAgICAgfVxuXG4gICAgICBpZiAodHlwZW9mIHNwcmluZ0xlbmd0aCAhPT0gJ251bWJlcicpIHtcbiAgICAgICAgc3ByaW5nTGVuZ3RoID0gLTE7IC8vIGFzc3VtZSBnbG9iYWwgY29uZmlndXJhdGlvblxuICAgICAgfVxuXG4gICAgICB2YXIgc3ByaW5nID0gbmV3IFNwcmluZyhib2R5MSwgYm9keTIsIHNwcmluZ0xlbmd0aCwgc3ByaW5nQ29lZmZpY2llbnQgPj0gMCA/IHNwcmluZ0NvZWZmaWNpZW50IDogLTEsIHNwcmluZ1dlaWdodCk7XG4gICAgICBzcHJpbmdzLnB1c2goc3ByaW5nKTtcblxuICAgICAgLy8gVE9ETzogY291bGQgbWFyayBzaW11bGF0b3IgYXMgZGlydHkuXG4gICAgICByZXR1cm4gc3ByaW5nO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBSZXR1cm5zIGFtb3VudCBvZiBtb3ZlbWVudCBwZXJmb3JtZWQgb24gbGFzdCBzdGVwKCkgY2FsbFxuICAgICAqL1xuICAgIGdldFRvdGFsTW92ZW1lbnQ6IGZ1bmN0aW9uICgpIHtcbiAgICAgIHJldHVybiB0b3RhbE1vdmVtZW50O1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBSZW1vdmVzIHNwcmluZyBmcm9tIHRoZSBzeXN0ZW1cbiAgICAgKlxuICAgICAqIEBwYXJhbSB7T2JqZWN0fSBzcHJpbmcgdG8gcmVtb3ZlLiBTcHJpbmcgaXMgYW4gb2JqZWN0IHJldHVybmVkIGJ5IGFkZFNwcmluZ1xuICAgICAqXG4gICAgICogQHJldHVybnMge0Jvb2xlYW59IHRydWUgaWYgc3ByaW5nIGZvdW5kIGFuZCByZW1vdmVkLiBmYWxzeSBvdGhlcndpc2U7XG4gICAgICovXG4gICAgcmVtb3ZlU3ByaW5nOiBmdW5jdGlvbiAoc3ByaW5nKSB7XG4gICAgICBpZiAoIXNwcmluZykgeyByZXR1cm47IH1cbiAgICAgIHZhciBpZHggPSBzcHJpbmdzLmluZGV4T2Yoc3ByaW5nKTtcbiAgICAgIGlmIChpZHggPiAtMSkge1xuICAgICAgICBzcHJpbmdzLnNwbGljZShpZHgsIDEpO1xuICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgIH1cbiAgICB9LFxuXG4gICAgZ2V0QmVzdE5ld0JvZHlQb3NpdGlvbjogZnVuY3Rpb24gKG5laWdoYm9ycykge1xuICAgICAgcmV0dXJuIGJvdW5kcy5nZXRCZXN0TmV3UG9zaXRpb24obmVpZ2hib3JzKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogUmV0dXJucyBib3VuZGluZyBib3ggd2hpY2ggY292ZXJzIGFsbCBib2RpZXNcbiAgICAgKi9cbiAgICBnZXRCQm94OiBmdW5jdGlvbiAoKSB7XG4gICAgICByZXR1cm4gYm91bmRzLmJveDtcbiAgICB9LFxuXG4gICAgZ3Jhdml0eTogZnVuY3Rpb24gKHZhbHVlKSB7XG4gICAgICBpZiAodmFsdWUgIT09IHVuZGVmaW5lZCkge1xuICAgICAgICBzZXR0aW5ncy5ncmF2aXR5ID0gdmFsdWU7XG4gICAgICAgIHF1YWRUcmVlLm9wdGlvbnMoe2dyYXZpdHk6IHZhbHVlfSk7XG4gICAgICAgIHJldHVybiB0aGlzO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0dXJuIHNldHRpbmdzLmdyYXZpdHk7XG4gICAgICB9XG4gICAgfSxcblxuICAgIHRoZXRhOiBmdW5jdGlvbiAodmFsdWUpIHtcbiAgICAgIGlmICh2YWx1ZSAhPT0gdW5kZWZpbmVkKSB7XG4gICAgICAgIHNldHRpbmdzLnRoZXRhID0gdmFsdWU7XG4gICAgICAgIHF1YWRUcmVlLm9wdGlvbnMoe3RoZXRhOiB2YWx1ZX0pO1xuICAgICAgICByZXR1cm4gdGhpcztcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHJldHVybiBzZXR0aW5ncy50aGV0YTtcbiAgICAgIH1cbiAgICB9XG4gIH07XG5cbiAgLy8gYWxsb3cgc2V0dGluZ3MgbW9kaWZpY2F0aW9uIHZpYSBwdWJsaWMgQVBJOlxuICBleHBvc2Uoc2V0dGluZ3MsIHB1YmxpY0FwaSk7XG5cbiAgZXZlbnRpZnkocHVibGljQXBpKTtcblxuICByZXR1cm4gcHVibGljQXBpO1xuXG4gIGZ1bmN0aW9uIGFjY3VtdWxhdGVGb3JjZXMoKSB7XG4gICAgLy8gQWNjdW11bGF0ZSBmb3JjZXMgYWN0aW5nIG9uIGJvZGllcy5cbiAgICB2YXIgYm9keSxcbiAgICAgICAgaSA9IGJvZGllcy5sZW5ndGg7XG5cbiAgICBpZiAoaSkge1xuICAgICAgLy8gb25seSBhZGQgYm9kaWVzIGlmIHRoZXJlIHRoZSBhcnJheSBpcyBub3QgZW1wdHk6XG4gICAgICBxdWFkVHJlZS5pbnNlcnRCb2RpZXMoYm9kaWVzKTsgLy8gcGVyZm9ybWFuY2U6IE8obiAqIGxvZyBuKVxuICAgICAgd2hpbGUgKGktLSkge1xuICAgICAgICBib2R5ID0gYm9kaWVzW2ldO1xuICAgICAgICAvLyBJZiBib2R5IGlzIHBpbm5lZCB0aGVyZSBpcyBubyBwb2ludCB1cGRhdGluZyBpdHMgZm9yY2VzIC0gaXQgc2hvdWxkXG4gICAgICAgIC8vIG5ldmVyIG1vdmU6XG4gICAgICAgIGlmICghYm9keS5pc1Bpbm5lZCkge1xuICAgICAgICAgIGJvZHkuZm9yY2UucmVzZXQoKTtcblxuICAgICAgICAgIHF1YWRUcmVlLnVwZGF0ZUJvZHlGb3JjZShib2R5KTtcbiAgICAgICAgICBkcmFnRm9yY2UudXBkYXRlKGJvZHkpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgaSA9IHNwcmluZ3MubGVuZ3RoO1xuICAgIHdoaWxlKGktLSkge1xuICAgICAgc3ByaW5nRm9yY2UudXBkYXRlKHNwcmluZ3NbaV0pO1xuICAgIH1cbiAgfVxufTtcbiIsIm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKGJvZGllcywgc2V0dGluZ3MpIHtcbiAgdmFyIHJhbmRvbSA9IHJlcXVpcmUoJ25ncmFwaC5yYW5kb20nKS5yYW5kb20oNDIpO1xuICB2YXIgYm91bmRpbmdCb3ggPSAgeyB4MTogMCwgeTE6IDAsIHgyOiAwLCB5MjogMCB9O1xuXG4gIHJldHVybiB7XG4gICAgYm94OiBib3VuZGluZ0JveCxcblxuICAgIHVwZGF0ZTogdXBkYXRlQm91bmRpbmdCb3gsXG5cbiAgICByZXNldCA6IGZ1bmN0aW9uICgpIHtcbiAgICAgIGJvdW5kaW5nQm94LngxID0gYm91bmRpbmdCb3gueTEgPSAwO1xuICAgICAgYm91bmRpbmdCb3gueDIgPSBib3VuZGluZ0JveC55MiA9IDA7XG4gICAgfSxcblxuICAgIGdldEJlc3ROZXdQb3NpdGlvbjogZnVuY3Rpb24gKG5laWdoYm9ycykge1xuICAgICAgdmFyIGdyYXBoUmVjdCA9IGJvdW5kaW5nQm94O1xuXG4gICAgICB2YXIgYmFzZVggPSAwLCBiYXNlWSA9IDA7XG5cbiAgICAgIGlmIChuZWlnaGJvcnMubGVuZ3RoKSB7XG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbmVpZ2hib3JzLmxlbmd0aDsgKytpKSB7XG4gICAgICAgICAgYmFzZVggKz0gbmVpZ2hib3JzW2ldLnBvcy54O1xuICAgICAgICAgIGJhc2VZICs9IG5laWdoYm9yc1tpXS5wb3MueTtcbiAgICAgICAgfVxuXG4gICAgICAgIGJhc2VYIC89IG5laWdoYm9ycy5sZW5ndGg7XG4gICAgICAgIGJhc2VZIC89IG5laWdoYm9ycy5sZW5ndGg7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBiYXNlWCA9IChncmFwaFJlY3QueDEgKyBncmFwaFJlY3QueDIpIC8gMjtcbiAgICAgICAgYmFzZVkgPSAoZ3JhcGhSZWN0LnkxICsgZ3JhcGhSZWN0LnkyKSAvIDI7XG4gICAgICB9XG5cbiAgICAgIHZhciBzcHJpbmdMZW5ndGggPSBzZXR0aW5ncy5zcHJpbmdMZW5ndGg7XG4gICAgICByZXR1cm4ge1xuICAgICAgICB4OiBiYXNlWCArIHJhbmRvbS5uZXh0KHNwcmluZ0xlbmd0aCkgLSBzcHJpbmdMZW5ndGggLyAyLFxuICAgICAgICB5OiBiYXNlWSArIHJhbmRvbS5uZXh0KHNwcmluZ0xlbmd0aCkgLSBzcHJpbmdMZW5ndGggLyAyXG4gICAgICB9O1xuICAgIH1cbiAgfTtcblxuICBmdW5jdGlvbiB1cGRhdGVCb3VuZGluZ0JveCgpIHtcbiAgICB2YXIgaSA9IGJvZGllcy5sZW5ndGg7XG4gICAgaWYgKGkgPT09IDApIHsgcmV0dXJuOyB9IC8vIGRvbid0IGhhdmUgdG8gd29yeSBoZXJlLlxuXG4gICAgdmFyIHgxID0gTnVtYmVyLk1BWF9WQUxVRSxcbiAgICAgICAgeTEgPSBOdW1iZXIuTUFYX1ZBTFVFLFxuICAgICAgICB4MiA9IE51bWJlci5NSU5fVkFMVUUsXG4gICAgICAgIHkyID0gTnVtYmVyLk1JTl9WQUxVRTtcblxuICAgIHdoaWxlKGktLSkge1xuICAgICAgLy8gdGhpcyBpcyBPKG4pLCBjb3VsZCBpdCBiZSBkb25lIGZhc3RlciB3aXRoIHF1YWR0cmVlP1xuICAgICAgLy8gaG93IGFib3V0IHBpbm5lZCBub2Rlcz9cbiAgICAgIHZhciBib2R5ID0gYm9kaWVzW2ldO1xuICAgICAgaWYgKGJvZHkuaXNQaW5uZWQpIHtcbiAgICAgICAgYm9keS5wb3MueCA9IGJvZHkucHJldlBvcy54O1xuICAgICAgICBib2R5LnBvcy55ID0gYm9keS5wcmV2UG9zLnk7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBib2R5LnByZXZQb3MueCA9IGJvZHkucG9zLng7XG4gICAgICAgIGJvZHkucHJldlBvcy55ID0gYm9keS5wb3MueTtcbiAgICAgIH1cbiAgICAgIGlmIChib2R5LnBvcy54IDwgeDEpIHtcbiAgICAgICAgeDEgPSBib2R5LnBvcy54O1xuICAgICAgfVxuICAgICAgaWYgKGJvZHkucG9zLnggPiB4Mikge1xuICAgICAgICB4MiA9IGJvZHkucG9zLng7XG4gICAgICB9XG4gICAgICBpZiAoYm9keS5wb3MueSA8IHkxKSB7XG4gICAgICAgIHkxID0gYm9keS5wb3MueTtcbiAgICAgIH1cbiAgICAgIGlmIChib2R5LnBvcy55ID4geTIpIHtcbiAgICAgICAgeTIgPSBib2R5LnBvcy55O1xuICAgICAgfVxuICAgIH1cblxuICAgIGJvdW5kaW5nQm94LngxID0geDE7XG4gICAgYm91bmRpbmdCb3gueDIgPSB4MjtcbiAgICBib3VuZGluZ0JveC55MSA9IHkxO1xuICAgIGJvdW5kaW5nQm94LnkyID0geTI7XG4gIH1cbn1cbiIsInZhciBwaHlzaWNzID0gcmVxdWlyZSgnbmdyYXBoLnBoeXNpY3MucHJpbWl0aXZlcycpO1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uKHBvcykge1xuICByZXR1cm4gbmV3IHBoeXNpY3MuQm9keShwb3MpO1xufVxuIiwiLyoqXG4gKiBSZXByZXNlbnRzIGRyYWcgZm9yY2UsIHdoaWNoIHJlZHVjZXMgZm9yY2UgdmFsdWUgb24gZWFjaCBzdGVwIGJ5IGdpdmVuXG4gKiBjb2VmZmljaWVudC5cbiAqXG4gKiBAcGFyYW0ge09iamVjdH0gb3B0aW9ucyBmb3IgdGhlIGRyYWcgZm9yY2VcbiAqIEBwYXJhbSB7TnVtYmVyPX0gb3B0aW9ucy5kcmFnQ29lZmYgZHJhZyBmb3JjZSBjb2VmZmljaWVudC4gMC4xIGJ5IGRlZmF1bHRcbiAqL1xubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAob3B0aW9ucykge1xuICB2YXIgbWVyZ2UgPSByZXF1aXJlKCduZ3JhcGgubWVyZ2UnKSxcbiAgICAgIGV4cG9zZSA9IHJlcXVpcmUoJ25ncmFwaC5leHBvc2UnKTtcblxuICBvcHRpb25zID0gbWVyZ2Uob3B0aW9ucywge1xuICAgIGRyYWdDb2VmZjogMC4wMlxuICB9KTtcblxuICB2YXIgYXBpID0ge1xuICAgIHVwZGF0ZSA6IGZ1bmN0aW9uIChib2R5KSB7XG4gICAgICBib2R5LmZvcmNlLnggLT0gb3B0aW9ucy5kcmFnQ29lZmYgKiBib2R5LnZlbG9jaXR5Lng7XG4gICAgICBib2R5LmZvcmNlLnkgLT0gb3B0aW9ucy5kcmFnQ29lZmYgKiBib2R5LnZlbG9jaXR5Lnk7XG4gICAgfVxuICB9O1xuXG4gIC8vIGxldCBlYXN5IGFjY2VzcyB0byBkcmFnQ29lZmY6XG4gIGV4cG9zZShvcHRpb25zLCBhcGksIFsnZHJhZ0NvZWZmJ10pO1xuXG4gIHJldHVybiBhcGk7XG59O1xuIiwiLyoqXG4gKiBQZXJmb3JtcyBmb3JjZXMgaW50ZWdyYXRpb24sIHVzaW5nIGdpdmVuIHRpbWVzdGVwLiBVc2VzIEV1bGVyIG1ldGhvZCB0byBzb2x2ZVxuICogZGlmZmVyZW50aWFsIGVxdWF0aW9uIChodHRwOi8vZW4ud2lraXBlZGlhLm9yZy93aWtpL0V1bGVyX21ldGhvZCApLlxuICpcbiAqIEByZXR1cm5zIHtOdW1iZXJ9IHNxdWFyZWQgZGlzdGFuY2Ugb2YgdG90YWwgcG9zaXRpb24gdXBkYXRlcy5cbiAqL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGludGVncmF0ZTtcblxuZnVuY3Rpb24gaW50ZWdyYXRlKGJvZGllcywgdGltZVN0ZXApIHtcbiAgdmFyIGR4ID0gMCwgdHggPSAwLFxuICAgICAgZHkgPSAwLCB0eSA9IDAsXG4gICAgICBpLFxuICAgICAgbWF4ID0gYm9kaWVzLmxlbmd0aDtcblxuICBpZiAobWF4ID09PSAwKSB7XG4gICAgcmV0dXJuIDA7XG4gIH1cblxuICBmb3IgKGkgPSAwOyBpIDwgbWF4OyArK2kpIHtcbiAgICB2YXIgYm9keSA9IGJvZGllc1tpXSxcbiAgICAgICAgY29lZmYgPSB0aW1lU3RlcCAvIGJvZHkubWFzcztcblxuICAgIGJvZHkudmVsb2NpdHkueCArPSBjb2VmZiAqIGJvZHkuZm9yY2UueDtcbiAgICBib2R5LnZlbG9jaXR5LnkgKz0gY29lZmYgKiBib2R5LmZvcmNlLnk7XG4gICAgdmFyIHZ4ID0gYm9keS52ZWxvY2l0eS54LFxuICAgICAgICB2eSA9IGJvZHkudmVsb2NpdHkueSxcbiAgICAgICAgdiA9IE1hdGguc3FydCh2eCAqIHZ4ICsgdnkgKiB2eSk7XG5cbiAgICBpZiAodiA+IDEpIHtcbiAgICAgIGJvZHkudmVsb2NpdHkueCA9IHZ4IC8gdjtcbiAgICAgIGJvZHkudmVsb2NpdHkueSA9IHZ5IC8gdjtcbiAgICB9XG5cbiAgICBkeCA9IHRpbWVTdGVwICogYm9keS52ZWxvY2l0eS54O1xuICAgIGR5ID0gdGltZVN0ZXAgKiBib2R5LnZlbG9jaXR5Lnk7XG5cbiAgICBib2R5LnBvcy54ICs9IGR4O1xuICAgIGJvZHkucG9zLnkgKz0gZHk7XG5cbiAgICB0eCArPSBNYXRoLmFicyhkeCk7IHR5ICs9IE1hdGguYWJzKGR5KTtcbiAgfVxuXG4gIHJldHVybiAodHggKiB0eCArIHR5ICogdHkpL21heDtcbn1cbiIsIm1vZHVsZS5leHBvcnRzID0gU3ByaW5nO1xuXG4vKipcbiAqIFJlcHJlc2VudHMgYSBwaHlzaWNhbCBzcHJpbmcuIFNwcmluZyBjb25uZWN0cyB0d28gYm9kaWVzLCBoYXMgcmVzdCBsZW5ndGhcbiAqIHN0aWZmbmVzcyBjb2VmZmljaWVudCBhbmQgb3B0aW9uYWwgd2VpZ2h0XG4gKi9cbmZ1bmN0aW9uIFNwcmluZyhmcm9tQm9keSwgdG9Cb2R5LCBsZW5ndGgsIGNvZWZmLCB3ZWlnaHQpIHtcbiAgICB0aGlzLmZyb20gPSBmcm9tQm9keTtcbiAgICB0aGlzLnRvID0gdG9Cb2R5O1xuICAgIHRoaXMubGVuZ3RoID0gbGVuZ3RoO1xuICAgIHRoaXMuY29lZmYgPSBjb2VmZjtcblxuICAgIHRoaXMud2VpZ2h0ID0gdHlwZW9mIHdlaWdodCA9PT0gJ251bWJlcicgPyB3ZWlnaHQgOiAxO1xufTtcbiIsIi8qKlxuICogUmVwcmVzZW50cyBzcHJpbmcgZm9yY2UsIHdoaWNoIHVwZGF0ZXMgZm9yY2VzIGFjdGluZyBvbiB0d28gYm9kaWVzLCBjb25udGVjdGVkXG4gKiBieSBhIHNwcmluZy5cbiAqXG4gKiBAcGFyYW0ge09iamVjdH0gb3B0aW9ucyBmb3IgdGhlIHNwcmluZyBmb3JjZVxuICogQHBhcmFtIHtOdW1iZXI9fSBvcHRpb25zLnNwcmluZ0NvZWZmIHNwcmluZyBmb3JjZSBjb2VmZmljaWVudC5cbiAqIEBwYXJhbSB7TnVtYmVyPX0gb3B0aW9ucy5zcHJpbmdMZW5ndGggZGVzaXJlZCBsZW5ndGggb2YgYSBzcHJpbmcgYXQgcmVzdC5cbiAqL1xubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAob3B0aW9ucykge1xuICB2YXIgbWVyZ2UgPSByZXF1aXJlKCduZ3JhcGgubWVyZ2UnKTtcbiAgdmFyIHJhbmRvbSA9IHJlcXVpcmUoJ25ncmFwaC5yYW5kb20nKS5yYW5kb20oNDIpO1xuICB2YXIgZXhwb3NlID0gcmVxdWlyZSgnbmdyYXBoLmV4cG9zZScpO1xuXG4gIG9wdGlvbnMgPSBtZXJnZShvcHRpb25zLCB7XG4gICAgc3ByaW5nQ29lZmY6IDAuMDAwMixcbiAgICBzcHJpbmdMZW5ndGg6IDgwXG4gIH0pO1xuXG4gIHZhciBhcGkgPSB7XG4gICAgLyoqXG4gICAgICogVXBzYXRlcyBmb3JjZXMgYWN0aW5nIG9uIGEgc3ByaW5nXG4gICAgICovXG4gICAgdXBkYXRlIDogZnVuY3Rpb24gKHNwcmluZykge1xuICAgICAgdmFyIGJvZHkxID0gc3ByaW5nLmZyb20sXG4gICAgICAgICAgYm9keTIgPSBzcHJpbmcudG8sXG4gICAgICAgICAgbGVuZ3RoID0gc3ByaW5nLmxlbmd0aCA8IDAgPyBvcHRpb25zLnNwcmluZ0xlbmd0aCA6IHNwcmluZy5sZW5ndGgsXG4gICAgICAgICAgZHggPSBib2R5Mi5wb3MueCAtIGJvZHkxLnBvcy54LFxuICAgICAgICAgIGR5ID0gYm9keTIucG9zLnkgLSBib2R5MS5wb3MueSxcbiAgICAgICAgICByID0gTWF0aC5zcXJ0KGR4ICogZHggKyBkeSAqIGR5KTtcblxuICAgICAgaWYgKHIgPT09IDApIHtcbiAgICAgICAgICBkeCA9IChyYW5kb20ubmV4dERvdWJsZSgpIC0gMC41KSAvIDUwO1xuICAgICAgICAgIGR5ID0gKHJhbmRvbS5uZXh0RG91YmxlKCkgLSAwLjUpIC8gNTA7XG4gICAgICAgICAgciA9IE1hdGguc3FydChkeCAqIGR4ICsgZHkgKiBkeSk7XG4gICAgICB9XG5cbiAgICAgIHZhciBkID0gciAtIGxlbmd0aDtcbiAgICAgIHZhciBjb2VmZiA9ICgoIXNwcmluZy5jb2VmZiB8fCBzcHJpbmcuY29lZmYgPCAwKSA/IG9wdGlvbnMuc3ByaW5nQ29lZmYgOiBzcHJpbmcuY29lZmYpICogZCAvIHIgKiBzcHJpbmcud2VpZ2h0O1xuXG4gICAgICBib2R5MS5mb3JjZS54ICs9IGNvZWZmICogZHg7XG4gICAgICBib2R5MS5mb3JjZS55ICs9IGNvZWZmICogZHk7XG5cbiAgICAgIGJvZHkyLmZvcmNlLnggLT0gY29lZmYgKiBkeDtcbiAgICAgIGJvZHkyLmZvcmNlLnkgLT0gY29lZmYgKiBkeTtcbiAgICB9XG4gIH07XG5cbiAgZXhwb3NlKG9wdGlvbnMsIGFwaSwgWydzcHJpbmdDb2VmZicsICdzcHJpbmdMZW5ndGgnXSk7XG4gIHJldHVybiBhcGk7XG59XG4iLCJtb2R1bGUuZXhwb3J0cyA9IHtcbiAgcmFuZG9tOiByYW5kb20sXG4gIHJhbmRvbUl0ZXJhdG9yOiByYW5kb21JdGVyYXRvclxufTtcblxuLyoqXG4gKiBDcmVhdGVzIHNlZWRlZCBQUk5HIHdpdGggdHdvIG1ldGhvZHM6XG4gKiAgIG5leHQoKSBhbmQgbmV4dERvdWJsZSgpXG4gKi9cbmZ1bmN0aW9uIHJhbmRvbShpbnB1dFNlZWQpIHtcbiAgdmFyIHNlZWQgPSB0eXBlb2YgaW5wdXRTZWVkID09PSAnbnVtYmVyJyA/IGlucHV0U2VlZCA6ICgrIG5ldyBEYXRlKCkpO1xuICB2YXIgcmFuZG9tRnVuYyA9IGZ1bmN0aW9uKCkge1xuICAgICAgLy8gUm9iZXJ0IEplbmtpbnMnIDMyIGJpdCBpbnRlZ2VyIGhhc2ggZnVuY3Rpb24uXG4gICAgICBzZWVkID0gKChzZWVkICsgMHg3ZWQ1NWQxNikgKyAoc2VlZCA8PCAxMikpICAmIDB4ZmZmZmZmZmY7XG4gICAgICBzZWVkID0gKChzZWVkIF4gMHhjNzYxYzIzYykgXiAoc2VlZCA+Pj4gMTkpKSAmIDB4ZmZmZmZmZmY7XG4gICAgICBzZWVkID0gKChzZWVkICsgMHgxNjU2NjdiMSkgKyAoc2VlZCA8PCA1KSkgICAmIDB4ZmZmZmZmZmY7XG4gICAgICBzZWVkID0gKChzZWVkICsgMHhkM2EyNjQ2YykgXiAoc2VlZCA8PCA5KSkgICAmIDB4ZmZmZmZmZmY7XG4gICAgICBzZWVkID0gKChzZWVkICsgMHhmZDcwNDZjNSkgKyAoc2VlZCA8PCAzKSkgICAmIDB4ZmZmZmZmZmY7XG4gICAgICBzZWVkID0gKChzZWVkIF4gMHhiNTVhNGYwOSkgXiAoc2VlZCA+Pj4gMTYpKSAmIDB4ZmZmZmZmZmY7XG4gICAgICByZXR1cm4gKHNlZWQgJiAweGZmZmZmZmYpIC8gMHgxMDAwMDAwMDtcbiAgfTtcblxuICByZXR1cm4ge1xuICAgICAgLyoqXG4gICAgICAgKiBHZW5lcmF0ZXMgcmFuZG9tIGludGVnZXIgbnVtYmVyIGluIHRoZSByYW5nZSBmcm9tIDAgKGluY2x1c2l2ZSkgdG8gbWF4VmFsdWUgKGV4Y2x1c2l2ZSlcbiAgICAgICAqXG4gICAgICAgKiBAcGFyYW0gbWF4VmFsdWUgTnVtYmVyIFJFUVVJUkVELiBPbW1pdHRpbmcgdGhpcyBudW1iZXIgd2lsbCByZXN1bHQgaW4gTmFOIHZhbHVlcyBmcm9tIFBSTkcuXG4gICAgICAgKi9cbiAgICAgIG5leHQgOiBmdW5jdGlvbiAobWF4VmFsdWUpIHtcbiAgICAgICAgICByZXR1cm4gTWF0aC5mbG9vcihyYW5kb21GdW5jKCkgKiBtYXhWYWx1ZSk7XG4gICAgICB9LFxuXG4gICAgICAvKipcbiAgICAgICAqIEdlbmVyYXRlcyByYW5kb20gZG91YmxlIG51bWJlciBpbiB0aGUgcmFuZ2UgZnJvbSAwIChpbmNsdXNpdmUpIHRvIDEgKGV4Y2x1c2l2ZSlcbiAgICAgICAqIFRoaXMgZnVuY3Rpb24gaXMgdGhlIHNhbWUgYXMgTWF0aC5yYW5kb20oKSAoZXhjZXB0IHRoYXQgaXQgY291bGQgYmUgc2VlZGVkKVxuICAgICAgICovXG4gICAgICBuZXh0RG91YmxlIDogZnVuY3Rpb24gKCkge1xuICAgICAgICAgIHJldHVybiByYW5kb21GdW5jKCk7XG4gICAgICB9XG4gIH07XG59XG5cbi8qXG4gKiBDcmVhdGVzIGl0ZXJhdG9yIG92ZXIgYXJyYXksIHdoaWNoIHJldHVybnMgaXRlbXMgb2YgYXJyYXkgaW4gcmFuZG9tIG9yZGVyXG4gKiBUaW1lIGNvbXBsZXhpdHkgaXMgZ3VhcmFudGVlZCB0byBiZSBPKG4pO1xuICovXG5mdW5jdGlvbiByYW5kb21JdGVyYXRvcihhcnJheSwgY3VzdG9tUmFuZG9tKSB7XG4gICAgdmFyIGxvY2FsUmFuZG9tID0gY3VzdG9tUmFuZG9tIHx8IHJhbmRvbSgpO1xuICAgIGlmICh0eXBlb2YgbG9jYWxSYW5kb20ubmV4dCAhPT0gJ2Z1bmN0aW9uJykge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKCdjdXN0b21SYW5kb20gZG9lcyBub3QgbWF0Y2ggZXhwZWN0ZWQgQVBJOiBuZXh0KCkgZnVuY3Rpb24gaXMgbWlzc2luZycpO1xuICAgIH1cblxuICAgIHJldHVybiB7XG4gICAgICAgIGZvckVhY2ggOiBmdW5jdGlvbiAoY2FsbGJhY2spIHtcbiAgICAgICAgICAgIHZhciBpLCBqLCB0O1xuICAgICAgICAgICAgZm9yIChpID0gYXJyYXkubGVuZ3RoIC0gMTsgaSA+IDA7IC0taSkge1xuICAgICAgICAgICAgICAgIGogPSBsb2NhbFJhbmRvbS5uZXh0KGkgKyAxKTsgLy8gaSBpbmNsdXNpdmVcbiAgICAgICAgICAgICAgICB0ID0gYXJyYXlbal07XG4gICAgICAgICAgICAgICAgYXJyYXlbal0gPSBhcnJheVtpXTtcbiAgICAgICAgICAgICAgICBhcnJheVtpXSA9IHQ7XG5cbiAgICAgICAgICAgICAgICBjYWxsYmFjayh0KTtcbiAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgaWYgKGFycmF5Lmxlbmd0aCkge1xuICAgICAgICAgICAgICAgIGNhbGxiYWNrKGFycmF5WzBdKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfSxcblxuICAgICAgICAvKipcbiAgICAgICAgICogU2h1ZmZsZXMgYXJyYXkgcmFuZG9tbHksIGluIHBsYWNlLlxuICAgICAgICAgKi9cbiAgICAgICAgc2h1ZmZsZSA6IGZ1bmN0aW9uICgpIHtcbiAgICAgICAgICAgIHZhciBpLCBqLCB0O1xuICAgICAgICAgICAgZm9yIChpID0gYXJyYXkubGVuZ3RoIC0gMTsgaSA+IDA7IC0taSkge1xuICAgICAgICAgICAgICAgIGogPSBsb2NhbFJhbmRvbS5uZXh0KGkgKyAxKTsgLy8gaSBpbmNsdXNpdmVcbiAgICAgICAgICAgICAgICB0ID0gYXJyYXlbal07XG4gICAgICAgICAgICAgICAgYXJyYXlbal0gPSBhcnJheVtpXTtcbiAgICAgICAgICAgICAgICBhcnJheVtpXSA9IHQ7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIHJldHVybiBhcnJheTtcbiAgICAgICAgfVxuICAgIH07XG59XG4iLCIvKipcbiAqIEBmaWxlT3ZlcnZpZXcgQ29udGFpbnMgZGVmaW5pdGlvbiBvZiB0aGUgY29yZSBncmFwaCBvYmplY3QuXG4gKi9cblxuLy8gVE9ETzogbmVlZCB0byBjaGFuZ2Ugc3RvcmFnZSBsYXllcjpcbi8vIDEuIEJlIGFibGUgdG8gZ2V0IGFsbCBub2RlcyBPKDEpXG4vLyAyLiBCZSBhYmxlIHRvIGdldCBudW1iZXIgb2YgbGlua3MgTygxKVxuXG4vKipcbiAqIEBleGFtcGxlXG4gKiAgdmFyIGdyYXBoID0gcmVxdWlyZSgnbmdyYXBoLmdyYXBoJykoKTtcbiAqICBncmFwaC5hZGROb2RlKDEpOyAgICAgLy8gZ3JhcGggaGFzIG9uZSBub2RlLlxuICogIGdyYXBoLmFkZExpbmsoMiwgMyk7ICAvLyBub3cgZ3JhcGggY29udGFpbnMgdGhyZWUgbm9kZXMgYW5kIG9uZSBsaW5rLlxuICpcbiAqL1xubW9kdWxlLmV4cG9ydHMgPSBjcmVhdGVHcmFwaDtcblxudmFyIGV2ZW50aWZ5ID0gcmVxdWlyZSgnbmdyYXBoLmV2ZW50cycpO1xuXG4vKipcbiAqIENyZWF0ZXMgYSBuZXcgZ3JhcGhcbiAqL1xuZnVuY3Rpb24gY3JlYXRlR3JhcGgob3B0aW9ucykge1xuICAvLyBHcmFwaCBzdHJ1Y3R1cmUgaXMgbWFpbnRhaW5lZCBhcyBkaWN0aW9uYXJ5IG9mIG5vZGVzXG4gIC8vIGFuZCBhcnJheSBvZiBsaW5rcy4gRWFjaCBub2RlIGhhcyAnbGlua3MnIHByb3BlcnR5IHdoaWNoXG4gIC8vIGhvbGQgYWxsIGxpbmtzIHJlbGF0ZWQgdG8gdGhhdCBub2RlLiBBbmQgZ2VuZXJhbCBsaW5rc1xuICAvLyBhcnJheSBpcyB1c2VkIHRvIHNwZWVkIHVwIGFsbCBsaW5rcyBlbnVtZXJhdGlvbi4gVGhpcyBpcyBpbmVmZmljaWVudFxuICAvLyBpbiB0ZXJtcyBvZiBtZW1vcnksIGJ1dCBzaW1wbGlmaWVzIGNvZGluZy5cbiAgb3B0aW9ucyA9IG9wdGlvbnMgfHwge307XG4gIGlmICgndW5pcXVlTGlua0lkJyBpbiBvcHRpb25zKSB7XG4gICAgY29uc29sZS53YXJuKFxuICAgICAgJ25ncmFwaC5ncmFwaDogU3RhcnRpbmcgZnJvbSB2ZXJzaW9uIDAuMTQgYHVuaXF1ZUxpbmtJZGAgaXMgZGVwcmVjYXRlZC5cXG4nICtcbiAgICAgICdVc2UgYG11bHRpZ3JhcGhgIG9wdGlvbiBpbnN0ZWFkXFxuJyxcbiAgICAgICdcXG4nLFxuICAgICAgJ05vdGU6IHRoZXJlIGlzIGFsc28gY2hhbmdlIGluIGRlZmF1bHQgYmVoYXZpb3I6IEZyb20gbm93IG93biBlYWNoIGdyYXBoXFxuJytcbiAgICAgICdpcyBjb25zaWRlcmVkIHRvIGJlIG5vdCBhIG11bHRpZ3JhcGggYnkgZGVmYXVsdCAoZWFjaCBlZGdlIGlzIHVuaXF1ZSkuJ1xuICAgICk7XG5cbiAgICBvcHRpb25zLm11bHRpZ3JhcGggPSBvcHRpb25zLnVuaXF1ZUxpbmtJZDtcbiAgfVxuXG4gIC8vIERlYXIgcmVhZGVyLCB0aGUgbm9uLW11bHRpZ3JhcGhzIGRvIG5vdCBndWFyYW50ZWUgdGhhdCB0aGVyZSBpcyBvbmx5XG4gIC8vIG9uZSBsaW5rIGZvciBhIGdpdmVuIHBhaXIgb2Ygbm9kZS4gV2hlbiB0aGlzIG9wdGlvbiBpcyBzZXQgdG8gZmFsc2VcbiAgLy8gd2UgY2FuIHNhdmUgc29tZSBtZW1vcnkgYW5kIENQVSAoMTglIGZhc3RlciBmb3Igbm9uLW11bHRpZ3JhcGgpO1xuICBpZiAob3B0aW9ucy5tdWx0aWdyYXBoID09PSB1bmRlZmluZWQpIG9wdGlvbnMubXVsdGlncmFwaCA9IGZhbHNlO1xuXG4gIHZhciBub2RlcyA9IHR5cGVvZiBPYmplY3QuY3JlYXRlID09PSAnZnVuY3Rpb24nID8gT2JqZWN0LmNyZWF0ZShudWxsKSA6IHt9LFxuICAgIGxpbmtzID0gW10sXG4gICAgLy8gSGFzaCBvZiBtdWx0aS1lZGdlcy4gVXNlZCB0byB0cmFjayBpZHMgb2YgZWRnZXMgYmV0d2VlbiBzYW1lIG5vZGVzXG4gICAgbXVsdGlFZGdlcyA9IHt9LFxuICAgIG5vZGVzQ291bnQgPSAwLFxuICAgIHN1c3BlbmRFdmVudHMgPSAwLFxuXG4gICAgZm9yRWFjaE5vZGUgPSBjcmVhdGVOb2RlSXRlcmF0b3IoKSxcbiAgICBjcmVhdGVMaW5rID0gb3B0aW9ucy5tdWx0aWdyYXBoID8gY3JlYXRlVW5pcXVlTGluayA6IGNyZWF0ZVNpbmdsZUxpbmssXG5cbiAgICAvLyBPdXIgZ3JhcGggQVBJIHByb3ZpZGVzIG1lYW5zIHRvIGxpc3RlbiB0byBncmFwaCBjaGFuZ2VzLiBVc2VycyBjYW4gc3Vic2NyaWJlXG4gICAgLy8gdG8gYmUgbm90aWZpZWQgYWJvdXQgY2hhbmdlcyBpbiB0aGUgZ3JhcGggYnkgdXNpbmcgYG9uYCBtZXRob2QuIEhvd2V2ZXJcbiAgICAvLyBpbiBzb21lIGNhc2VzIHRoZXkgZG9uJ3QgdXNlIGl0LiBUbyBhdm9pZCB1bm5lY2Vzc2FyeSBtZW1vcnkgY29uc3VtcHRpb25cbiAgICAvLyB3ZSB3aWxsIG5vdCByZWNvcmQgZ3JhcGggY2hhbmdlcyB1bnRpbCB3ZSBoYXZlIGF0IGxlYXN0IG9uZSBzdWJzY3JpYmVyLlxuICAgIC8vIENvZGUgYmVsb3cgc3VwcG9ydHMgdGhpcyBvcHRpbWl6YXRpb24uXG4gICAgLy9cbiAgICAvLyBBY2N1bXVsYXRlcyBhbGwgY2hhbmdlcyBtYWRlIGR1cmluZyBncmFwaCB1cGRhdGVzLlxuICAgIC8vIEVhY2ggY2hhbmdlIGVsZW1lbnQgY29udGFpbnM6XG4gICAgLy8gIGNoYW5nZVR5cGUgLSBvbmUgb2YgdGhlIHN0cmluZ3M6ICdhZGQnLCAncmVtb3ZlJyBvciAndXBkYXRlJztcbiAgICAvLyAgbm9kZSAtIGlmIGNoYW5nZSBpcyByZWxhdGVkIHRvIG5vZGUgdGhpcyBwcm9wZXJ0eSBpcyBzZXQgdG8gY2hhbmdlZCBncmFwaCdzIG5vZGU7XG4gICAgLy8gIGxpbmsgLSBpZiBjaGFuZ2UgaXMgcmVsYXRlZCB0byBsaW5rIHRoaXMgcHJvcGVydHkgaXMgc2V0IHRvIGNoYW5nZWQgZ3JhcGgncyBsaW5rO1xuICAgIGNoYW5nZXMgPSBbXSxcbiAgICByZWNvcmRMaW5rQ2hhbmdlID0gbm9vcCxcbiAgICByZWNvcmROb2RlQ2hhbmdlID0gbm9vcCxcbiAgICBlbnRlck1vZGlmaWNhdGlvbiA9IG5vb3AsXG4gICAgZXhpdE1vZGlmaWNhdGlvbiA9IG5vb3A7XG5cbiAgLy8gdGhpcyBpcyBvdXIgcHVibGljIEFQSTpcbiAgdmFyIGdyYXBoUGFydCA9IHtcbiAgICAvKipcbiAgICAgKiBBZGRzIG5vZGUgdG8gdGhlIGdyYXBoLiBJZiBub2RlIHdpdGggZ2l2ZW4gaWQgYWxyZWFkeSBleGlzdHMgaW4gdGhlIGdyYXBoXG4gICAgICogaXRzIGRhdGEgaXMgZXh0ZW5kZWQgd2l0aCB3aGF0ZXZlciBjb21lcyBpbiAnZGF0YScgYXJndW1lbnQuXG4gICAgICpcbiAgICAgKiBAcGFyYW0gbm9kZUlkIHRoZSBub2RlJ3MgaWRlbnRpZmllci4gQSBzdHJpbmcgb3IgbnVtYmVyIGlzIHByZWZlcnJlZC5cbiAgICAgKiBAcGFyYW0gW2RhdGFdIGFkZGl0aW9uYWwgZGF0YSBmb3IgdGhlIG5vZGUgYmVpbmcgYWRkZWQuIElmIG5vZGUgYWxyZWFkeVxuICAgICAqICAgZXhpc3RzIGl0cyBkYXRhIG9iamVjdCBpcyBhdWdtZW50ZWQgd2l0aCB0aGUgbmV3IG9uZS5cbiAgICAgKlxuICAgICAqIEByZXR1cm4ge25vZGV9IFRoZSBuZXdseSBhZGRlZCBub2RlIG9yIG5vZGUgd2l0aCBnaXZlbiBpZCBpZiBpdCBhbHJlYWR5IGV4aXN0cy5cbiAgICAgKi9cbiAgICBhZGROb2RlOiBhZGROb2RlLFxuXG4gICAgLyoqXG4gICAgICogQWRkcyBhIGxpbmsgdG8gdGhlIGdyYXBoLiBUaGUgZnVuY3Rpb24gYWx3YXlzIGNyZWF0ZSBhIG5ld1xuICAgICAqIGxpbmsgYmV0d2VlbiB0d28gbm9kZXMuIElmIG9uZSBvZiB0aGUgbm9kZXMgZG9lcyBub3QgZXhpc3RzXG4gICAgICogYSBuZXcgbm9kZSBpcyBjcmVhdGVkLlxuICAgICAqXG4gICAgICogQHBhcmFtIGZyb21JZCBsaW5rIHN0YXJ0IG5vZGUgaWQ7XG4gICAgICogQHBhcmFtIHRvSWQgbGluayBlbmQgbm9kZSBpZDtcbiAgICAgKiBAcGFyYW0gW2RhdGFdIGFkZGl0aW9uYWwgZGF0YSB0byBiZSBzZXQgb24gdGhlIG5ldyBsaW5rO1xuICAgICAqXG4gICAgICogQHJldHVybiB7bGlua30gVGhlIG5ld2x5IGNyZWF0ZWQgbGlua1xuICAgICAqL1xuICAgIGFkZExpbms6IGFkZExpbmssXG5cbiAgICAvKipcbiAgICAgKiBSZW1vdmVzIGxpbmsgZnJvbSB0aGUgZ3JhcGguIElmIGxpbmsgZG9lcyBub3QgZXhpc3QgZG9lcyBub3RoaW5nLlxuICAgICAqXG4gICAgICogQHBhcmFtIGxpbmsgLSBvYmplY3QgcmV0dXJuZWQgYnkgYWRkTGluaygpIG9yIGdldExpbmtzKCkgbWV0aG9kcy5cbiAgICAgKlxuICAgICAqIEByZXR1cm5zIHRydWUgaWYgbGluayB3YXMgcmVtb3ZlZDsgZmFsc2Ugb3RoZXJ3aXNlLlxuICAgICAqL1xuICAgIHJlbW92ZUxpbms6IHJlbW92ZUxpbmssXG5cbiAgICAvKipcbiAgICAgKiBSZW1vdmVzIG5vZGUgd2l0aCBnaXZlbiBpZCBmcm9tIHRoZSBncmFwaC4gSWYgbm9kZSBkb2VzIG5vdCBleGlzdCBpbiB0aGUgZ3JhcGhcbiAgICAgKiBkb2VzIG5vdGhpbmcuXG4gICAgICpcbiAgICAgKiBAcGFyYW0gbm9kZUlkIG5vZGUncyBpZGVudGlmaWVyIHBhc3NlZCB0byBhZGROb2RlKCkgZnVuY3Rpb24uXG4gICAgICpcbiAgICAgKiBAcmV0dXJucyB0cnVlIGlmIG5vZGUgd2FzIHJlbW92ZWQ7IGZhbHNlIG90aGVyd2lzZS5cbiAgICAgKi9cbiAgICByZW1vdmVOb2RlOiByZW1vdmVOb2RlLFxuXG4gICAgLyoqXG4gICAgICogR2V0cyBub2RlIHdpdGggZ2l2ZW4gaWRlbnRpZmllci4gSWYgbm9kZSBkb2VzIG5vdCBleGlzdCB1bmRlZmluZWQgdmFsdWUgaXMgcmV0dXJuZWQuXG4gICAgICpcbiAgICAgKiBAcGFyYW0gbm9kZUlkIHJlcXVlc3RlZCBub2RlIGlkZW50aWZpZXI7XG4gICAgICpcbiAgICAgKiBAcmV0dXJuIHtub2RlfSBpbiB3aXRoIHJlcXVlc3RlZCBpZGVudGlmaWVyIG9yIHVuZGVmaW5lZCBpZiBubyBzdWNoIG5vZGUgZXhpc3RzLlxuICAgICAqL1xuICAgIGdldE5vZGU6IGdldE5vZGUsXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIG51bWJlciBvZiBub2RlcyBpbiB0aGlzIGdyYXBoLlxuICAgICAqXG4gICAgICogQHJldHVybiBudW1iZXIgb2Ygbm9kZXMgaW4gdGhlIGdyYXBoLlxuICAgICAqL1xuICAgIGdldE5vZGVzQ291bnQ6IGZ1bmN0aW9uICgpIHtcbiAgICAgIHJldHVybiBub2Rlc0NvdW50O1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRvdGFsIG51bWJlciBvZiBsaW5rcyBpbiB0aGUgZ3JhcGguXG4gICAgICovXG4gICAgZ2V0TGlua3NDb3VudDogZnVuY3Rpb24gKCkge1xuICAgICAgcmV0dXJuIGxpbmtzLmxlbmd0aDtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogR2V0cyBhbGwgbGlua3MgKGluYm91bmQgYW5kIG91dGJvdW5kKSBmcm9tIHRoZSBub2RlIHdpdGggZ2l2ZW4gaWQuXG4gICAgICogSWYgbm9kZSB3aXRoIGdpdmVuIGlkIGlzIG5vdCBmb3VuZCBudWxsIGlzIHJldHVybmVkLlxuICAgICAqXG4gICAgICogQHBhcmFtIG5vZGVJZCByZXF1ZXN0ZWQgbm9kZSBpZGVudGlmaWVyLlxuICAgICAqXG4gICAgICogQHJldHVybiBBcnJheSBvZiBsaW5rcyBmcm9tIGFuZCB0byByZXF1ZXN0ZWQgbm9kZSBpZiBzdWNoIG5vZGUgZXhpc3RzO1xuICAgICAqICAgb3RoZXJ3aXNlIG51bGwgaXMgcmV0dXJuZWQuXG4gICAgICovXG4gICAgZ2V0TGlua3M6IGdldExpbmtzLFxuXG4gICAgLyoqXG4gICAgICogSW52b2tlcyBjYWxsYmFjayBvbiBlYWNoIG5vZGUgb2YgdGhlIGdyYXBoLlxuICAgICAqXG4gICAgICogQHBhcmFtIHtGdW5jdGlvbihub2RlKX0gY2FsbGJhY2sgRnVuY3Rpb24gdG8gYmUgaW52b2tlZC4gVGhlIGZ1bmN0aW9uXG4gICAgICogICBpcyBwYXNzZWQgb25lIGFyZ3VtZW50OiB2aXNpdGVkIG5vZGUuXG4gICAgICovXG4gICAgZm9yRWFjaE5vZGU6IGZvckVhY2hOb2RlLFxuXG4gICAgLyoqXG4gICAgICogSW52b2tlcyBjYWxsYmFjayBvbiBldmVyeSBsaW5rZWQgKGFkamFjZW50KSBub2RlIHRvIHRoZSBnaXZlbiBvbmUuXG4gICAgICpcbiAgICAgKiBAcGFyYW0gbm9kZUlkIElkZW50aWZpZXIgb2YgdGhlIHJlcXVlc3RlZCBub2RlLlxuICAgICAqIEBwYXJhbSB7RnVuY3Rpb24obm9kZSwgbGluayl9IGNhbGxiYWNrIEZ1bmN0aW9uIHRvIGJlIGNhbGxlZCBvbiBhbGwgbGlua2VkIG5vZGVzLlxuICAgICAqICAgVGhlIGZ1bmN0aW9uIGlzIHBhc3NlZCB0d28gcGFyYW1ldGVyczogYWRqYWNlbnQgbm9kZSBhbmQgbGluayBvYmplY3QgaXRzZWxmLlxuICAgICAqIEBwYXJhbSBvcmllbnRlZCBpZiB0cnVlIGdyYXBoIHRyZWF0ZWQgYXMgb3JpZW50ZWQuXG4gICAgICovXG4gICAgZm9yRWFjaExpbmtlZE5vZGU6IGZvckVhY2hMaW5rZWROb2RlLFxuXG4gICAgLyoqXG4gICAgICogRW51bWVyYXRlcyBhbGwgbGlua3MgaW4gdGhlIGdyYXBoXG4gICAgICpcbiAgICAgKiBAcGFyYW0ge0Z1bmN0aW9uKGxpbmspfSBjYWxsYmFjayBGdW5jdGlvbiB0byBiZSBjYWxsZWQgb24gYWxsIGxpbmtzIGluIHRoZSBncmFwaC5cbiAgICAgKiAgIFRoZSBmdW5jdGlvbiBpcyBwYXNzZWQgb25lIHBhcmFtZXRlcjogZ3JhcGgncyBsaW5rIG9iamVjdC5cbiAgICAgKlxuICAgICAqIExpbmsgb2JqZWN0IGNvbnRhaW5zIGF0IGxlYXN0IHRoZSBmb2xsb3dpbmcgZmllbGRzOlxuICAgICAqICBmcm9tSWQgLSBub2RlIGlkIHdoZXJlIGxpbmsgc3RhcnRzO1xuICAgICAqICB0b0lkIC0gbm9kZSBpZCB3aGVyZSBsaW5rIGVuZHMsXG4gICAgICogIGRhdGEgLSBhZGRpdGlvbmFsIGRhdGEgcGFzc2VkIHRvIGdyYXBoLmFkZExpbmsoKSBtZXRob2QuXG4gICAgICovXG4gICAgZm9yRWFjaExpbms6IGZvckVhY2hMaW5rLFxuXG4gICAgLyoqXG4gICAgICogU3VzcGVuZCBhbGwgbm90aWZpY2F0aW9ucyBhYm91dCBncmFwaCBjaGFuZ2VzIHVudGlsXG4gICAgICogZW5kVXBkYXRlIGlzIGNhbGxlZC5cbiAgICAgKi9cbiAgICBiZWdpblVwZGF0ZTogZW50ZXJNb2RpZmljYXRpb24sXG5cbiAgICAvKipcbiAgICAgKiBSZXN1bWVzIGFsbCBub3RpZmljYXRpb25zIGFib3V0IGdyYXBoIGNoYW5nZXMgYW5kIGZpcmVzXG4gICAgICogZ3JhcGggJ2NoYW5nZWQnIGV2ZW50IGluIGNhc2UgdGhlcmUgYXJlIGFueSBwZW5kaW5nIGNoYW5nZXMuXG4gICAgICovXG4gICAgZW5kVXBkYXRlOiBleGl0TW9kaWZpY2F0aW9uLFxuXG4gICAgLyoqXG4gICAgICogUmVtb3ZlcyBhbGwgbm9kZXMgYW5kIGxpbmtzIGZyb20gdGhlIGdyYXBoLlxuICAgICAqL1xuICAgIGNsZWFyOiBjbGVhcixcblxuICAgIC8qKlxuICAgICAqIERldGVjdHMgd2hldGhlciB0aGVyZSBpcyBhIGxpbmsgYmV0d2VlbiB0d28gbm9kZXMuXG4gICAgICogT3BlcmF0aW9uIGNvbXBsZXhpdHkgaXMgTyhuKSB3aGVyZSBuIC0gbnVtYmVyIG9mIGxpbmtzIG9mIGEgbm9kZS5cbiAgICAgKiBOT1RFOiB0aGlzIGZ1bmN0aW9uIGlzIHN5bm9uaW0gZm9yIGdldExpbmsoKVxuICAgICAqXG4gICAgICogQHJldHVybnMgbGluayBpZiB0aGVyZSBpcyBvbmUuIG51bGwgb3RoZXJ3aXNlLlxuICAgICAqL1xuICAgIGhhc0xpbms6IGdldExpbmssXG5cbiAgICAvKipcbiAgICAgKiBEZXRlY3RzIHdoZXRoZXIgdGhlcmUgaXMgYSBub2RlIHdpdGggZ2l2ZW4gaWRcbiAgICAgKiBcbiAgICAgKiBPcGVyYXRpb24gY29tcGxleGl0eSBpcyBPKDEpXG4gICAgICogTk9URTogdGhpcyBmdW5jdGlvbiBpcyBzeW5vbmltIGZvciBnZXROb2RlKClcbiAgICAgKlxuICAgICAqIEByZXR1cm5zIG5vZGUgaWYgdGhlcmUgaXMgb25lOyBGYWxzeSB2YWx1ZSBvdGhlcndpc2UuXG4gICAgICovXG4gICAgaGFzTm9kZTogZ2V0Tm9kZSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgYW4gZWRnZSBiZXR3ZWVuIHR3byBub2Rlcy5cbiAgICAgKiBPcGVyYXRpb24gY29tcGxleGl0eSBpcyBPKG4pIHdoZXJlIG4gLSBudW1iZXIgb2YgbGlua3Mgb2YgYSBub2RlLlxuICAgICAqXG4gICAgICogQHBhcmFtIHtzdHJpbmd9IGZyb21JZCBsaW5rIHN0YXJ0IGlkZW50aWZpZXJcbiAgICAgKiBAcGFyYW0ge3N0cmluZ30gdG9JZCBsaW5rIGVuZCBpZGVudGlmaWVyXG4gICAgICpcbiAgICAgKiBAcmV0dXJucyBsaW5rIGlmIHRoZXJlIGlzIG9uZS4gbnVsbCBvdGhlcndpc2UuXG4gICAgICovXG4gICAgZ2V0TGluazogZ2V0TGlua1xuICB9O1xuXG4gIC8vIHRoaXMgd2lsbCBhZGQgYG9uKClgIGFuZCBgZmlyZSgpYCBtZXRob2RzLlxuICBldmVudGlmeShncmFwaFBhcnQpO1xuXG4gIG1vbml0b3JTdWJzY3JpYmVycygpO1xuXG4gIHJldHVybiBncmFwaFBhcnQ7XG5cbiAgZnVuY3Rpb24gbW9uaXRvclN1YnNjcmliZXJzKCkge1xuICAgIHZhciByZWFsT24gPSBncmFwaFBhcnQub247XG5cbiAgICAvLyByZXBsYWNlIHJlYWwgYG9uYCB3aXRoIG91ciB0ZW1wb3Jhcnkgb24sIHdoaWNoIHdpbGwgdHJpZ2dlciBjaGFuZ2VcbiAgICAvLyBtb2RpZmljYXRpb24gbW9uaXRvcmluZzpcbiAgICBncmFwaFBhcnQub24gPSBvbjtcblxuICAgIGZ1bmN0aW9uIG9uKCkge1xuICAgICAgLy8gbm93IGl0J3MgdGltZSB0byBzdGFydCB0cmFja2luZyBzdHVmZjpcbiAgICAgIGdyYXBoUGFydC5iZWdpblVwZGF0ZSA9IGVudGVyTW9kaWZpY2F0aW9uID0gZW50ZXJNb2RpZmljYXRpb25SZWFsO1xuICAgICAgZ3JhcGhQYXJ0LmVuZFVwZGF0ZSA9IGV4aXRNb2RpZmljYXRpb24gPSBleGl0TW9kaWZpY2F0aW9uUmVhbDtcbiAgICAgIHJlY29yZExpbmtDaGFuZ2UgPSByZWNvcmRMaW5rQ2hhbmdlUmVhbDtcbiAgICAgIHJlY29yZE5vZGVDaGFuZ2UgPSByZWNvcmROb2RlQ2hhbmdlUmVhbDtcblxuICAgICAgLy8gdGhpcyB3aWxsIHJlcGxhY2UgY3VycmVudCBgb25gIG1ldGhvZCB3aXRoIHJlYWwgcHViL3N1YiBmcm9tIGBldmVudGlmeWAuXG4gICAgICBncmFwaFBhcnQub24gPSByZWFsT247XG4gICAgICAvLyBkZWxlZ2F0ZSB0byByZWFsIGBvbmAgaGFuZGxlcjpcbiAgICAgIHJldHVybiByZWFsT24uYXBwbHkoZ3JhcGhQYXJ0LCBhcmd1bWVudHMpO1xuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIHJlY29yZExpbmtDaGFuZ2VSZWFsKGxpbmssIGNoYW5nZVR5cGUpIHtcbiAgICBjaGFuZ2VzLnB1c2goe1xuICAgICAgbGluazogbGluayxcbiAgICAgIGNoYW5nZVR5cGU6IGNoYW5nZVR5cGVcbiAgICB9KTtcbiAgfVxuXG4gIGZ1bmN0aW9uIHJlY29yZE5vZGVDaGFuZ2VSZWFsKG5vZGUsIGNoYW5nZVR5cGUpIHtcbiAgICBjaGFuZ2VzLnB1c2goe1xuICAgICAgbm9kZTogbm9kZSxcbiAgICAgIGNoYW5nZVR5cGU6IGNoYW5nZVR5cGVcbiAgICB9KTtcbiAgfVxuXG4gIGZ1bmN0aW9uIGFkZE5vZGUobm9kZUlkLCBkYXRhKSB7XG4gICAgaWYgKG5vZGVJZCA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ0ludmFsaWQgbm9kZSBpZGVudGlmaWVyJyk7XG4gICAgfVxuXG4gICAgZW50ZXJNb2RpZmljYXRpb24oKTtcblxuICAgIHZhciBub2RlID0gZ2V0Tm9kZShub2RlSWQpO1xuICAgIGlmICghbm9kZSkge1xuICAgICAgbm9kZSA9IG5ldyBOb2RlKG5vZGVJZCwgZGF0YSk7XG4gICAgICBub2Rlc0NvdW50Kys7XG4gICAgICByZWNvcmROb2RlQ2hhbmdlKG5vZGUsICdhZGQnKTtcbiAgICB9IGVsc2Uge1xuICAgICAgbm9kZS5kYXRhID0gZGF0YTtcbiAgICAgIHJlY29yZE5vZGVDaGFuZ2Uobm9kZSwgJ3VwZGF0ZScpO1xuICAgIH1cblxuICAgIG5vZGVzW25vZGVJZF0gPSBub2RlO1xuXG4gICAgZXhpdE1vZGlmaWNhdGlvbigpO1xuICAgIHJldHVybiBub2RlO1xuICB9XG5cbiAgZnVuY3Rpb24gZ2V0Tm9kZShub2RlSWQpIHtcbiAgICByZXR1cm4gbm9kZXNbbm9kZUlkXTtcbiAgfVxuXG4gIGZ1bmN0aW9uIHJlbW92ZU5vZGUobm9kZUlkKSB7XG4gICAgdmFyIG5vZGUgPSBnZXROb2RlKG5vZGVJZCk7XG4gICAgaWYgKCFub2RlKSB7XG4gICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxuXG4gICAgZW50ZXJNb2RpZmljYXRpb24oKTtcblxuICAgIHZhciBwcmV2TGlua3MgPSBub2RlLmxpbmtzO1xuICAgIGlmIChwcmV2TGlua3MpIHtcbiAgICAgIG5vZGUubGlua3MgPSBudWxsO1xuICAgICAgZm9yKHZhciBpID0gMDsgaSA8IHByZXZMaW5rcy5sZW5ndGg7ICsraSkge1xuICAgICAgICByZW1vdmVMaW5rKHByZXZMaW5rc1tpXSk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgZGVsZXRlIG5vZGVzW25vZGVJZF07XG4gICAgbm9kZXNDb3VudC0tO1xuXG4gICAgcmVjb3JkTm9kZUNoYW5nZShub2RlLCAncmVtb3ZlJyk7XG5cbiAgICBleGl0TW9kaWZpY2F0aW9uKCk7XG5cbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG5cbiAgZnVuY3Rpb24gYWRkTGluayhmcm9tSWQsIHRvSWQsIGRhdGEpIHtcbiAgICBlbnRlck1vZGlmaWNhdGlvbigpO1xuXG4gICAgdmFyIGZyb21Ob2RlID0gZ2V0Tm9kZShmcm9tSWQpIHx8IGFkZE5vZGUoZnJvbUlkKTtcbiAgICB2YXIgdG9Ob2RlID0gZ2V0Tm9kZSh0b0lkKSB8fCBhZGROb2RlKHRvSWQpO1xuXG4gICAgdmFyIGxpbmsgPSBjcmVhdGVMaW5rKGZyb21JZCwgdG9JZCwgZGF0YSk7XG5cbiAgICBsaW5rcy5wdXNoKGxpbmspO1xuXG4gICAgLy8gVE9ETzogdGhpcyBpcyBub3QgY29vbC4gT24gbGFyZ2UgZ3JhcGhzIHBvdGVudGlhbGx5IHdvdWxkIGNvbnN1bWUgbW9yZSBtZW1vcnkuXG4gICAgYWRkTGlua1RvTm9kZShmcm9tTm9kZSwgbGluayk7XG4gICAgaWYgKGZyb21JZCAhPT0gdG9JZCkge1xuICAgICAgLy8gbWFrZSBzdXJlIHdlIGFyZSBub3QgZHVwbGljYXRpbmcgbGlua3MgZm9yIHNlbGYtbG9vcHNcbiAgICAgIGFkZExpbmtUb05vZGUodG9Ob2RlLCBsaW5rKTtcbiAgICB9XG5cbiAgICByZWNvcmRMaW5rQ2hhbmdlKGxpbmssICdhZGQnKTtcblxuICAgIGV4aXRNb2RpZmljYXRpb24oKTtcblxuICAgIHJldHVybiBsaW5rO1xuICB9XG5cbiAgZnVuY3Rpb24gY3JlYXRlU2luZ2xlTGluayhmcm9tSWQsIHRvSWQsIGRhdGEpIHtcbiAgICB2YXIgbGlua0lkID0gbWFrZUxpbmtJZChmcm9tSWQsIHRvSWQpO1xuICAgIHJldHVybiBuZXcgTGluayhmcm9tSWQsIHRvSWQsIGRhdGEsIGxpbmtJZCk7XG4gIH1cblxuICBmdW5jdGlvbiBjcmVhdGVVbmlxdWVMaW5rKGZyb21JZCwgdG9JZCwgZGF0YSkge1xuICAgIC8vIFRPRE86IEdldCByaWQgb2YgdGhpcyBtZXRob2QuXG4gICAgdmFyIGxpbmtJZCA9IG1ha2VMaW5rSWQoZnJvbUlkLCB0b0lkKTtcbiAgICB2YXIgaXNNdWx0aUVkZ2UgPSBtdWx0aUVkZ2VzLmhhc093blByb3BlcnR5KGxpbmtJZCk7XG4gICAgaWYgKGlzTXVsdGlFZGdlIHx8IGdldExpbmsoZnJvbUlkLCB0b0lkKSkge1xuICAgICAgaWYgKCFpc011bHRpRWRnZSkge1xuICAgICAgICBtdWx0aUVkZ2VzW2xpbmtJZF0gPSAwO1xuICAgICAgfVxuICAgICAgdmFyIHN1ZmZpeCA9ICdAJyArICgrK211bHRpRWRnZXNbbGlua0lkXSk7XG4gICAgICBsaW5rSWQgPSBtYWtlTGlua0lkKGZyb21JZCArIHN1ZmZpeCwgdG9JZCArIHN1ZmZpeCk7XG4gICAgfVxuXG4gICAgcmV0dXJuIG5ldyBMaW5rKGZyb21JZCwgdG9JZCwgZGF0YSwgbGlua0lkKTtcbiAgfVxuXG4gIGZ1bmN0aW9uIGdldExpbmtzKG5vZGVJZCkge1xuICAgIHZhciBub2RlID0gZ2V0Tm9kZShub2RlSWQpO1xuICAgIHJldHVybiBub2RlID8gbm9kZS5saW5rcyA6IG51bGw7XG4gIH1cblxuICBmdW5jdGlvbiByZW1vdmVMaW5rKGxpbmspIHtcbiAgICBpZiAoIWxpbmspIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG4gICAgdmFyIGlkeCA9IGluZGV4T2ZFbGVtZW50SW5BcnJheShsaW5rLCBsaW5rcyk7XG4gICAgaWYgKGlkeCA8IDApIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICBlbnRlck1vZGlmaWNhdGlvbigpO1xuXG4gICAgbGlua3Muc3BsaWNlKGlkeCwgMSk7XG5cbiAgICB2YXIgZnJvbU5vZGUgPSBnZXROb2RlKGxpbmsuZnJvbUlkKTtcbiAgICB2YXIgdG9Ob2RlID0gZ2V0Tm9kZShsaW5rLnRvSWQpO1xuXG4gICAgaWYgKGZyb21Ob2RlKSB7XG4gICAgICBpZHggPSBpbmRleE9mRWxlbWVudEluQXJyYXkobGluaywgZnJvbU5vZGUubGlua3MpO1xuICAgICAgaWYgKGlkeCA+PSAwKSB7XG4gICAgICAgIGZyb21Ob2RlLmxpbmtzLnNwbGljZShpZHgsIDEpO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmICh0b05vZGUpIHtcbiAgICAgIGlkeCA9IGluZGV4T2ZFbGVtZW50SW5BcnJheShsaW5rLCB0b05vZGUubGlua3MpO1xuICAgICAgaWYgKGlkeCA+PSAwKSB7XG4gICAgICAgIHRvTm9kZS5saW5rcy5zcGxpY2UoaWR4LCAxKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICByZWNvcmRMaW5rQ2hhbmdlKGxpbmssICdyZW1vdmUnKTtcblxuICAgIGV4aXRNb2RpZmljYXRpb24oKTtcblxuICAgIHJldHVybiB0cnVlO1xuICB9XG5cbiAgZnVuY3Rpb24gZ2V0TGluayhmcm9tTm9kZUlkLCB0b05vZGVJZCkge1xuICAgIC8vIFRPRE86IFVzZSBzb3J0ZWQgbGlua3MgdG8gc3BlZWQgdGhpcyB1cFxuICAgIHZhciBub2RlID0gZ2V0Tm9kZShmcm9tTm9kZUlkKSxcbiAgICAgIGk7XG4gICAgaWYgKCFub2RlIHx8ICFub2RlLmxpbmtzKSB7XG4gICAgICByZXR1cm4gbnVsbDtcbiAgICB9XG5cbiAgICBmb3IgKGkgPSAwOyBpIDwgbm9kZS5saW5rcy5sZW5ndGg7ICsraSkge1xuICAgICAgdmFyIGxpbmsgPSBub2RlLmxpbmtzW2ldO1xuICAgICAgaWYgKGxpbmsuZnJvbUlkID09PSBmcm9tTm9kZUlkICYmIGxpbmsudG9JZCA9PT0gdG9Ob2RlSWQpIHtcbiAgICAgICAgcmV0dXJuIGxpbms7XG4gICAgICB9XG4gICAgfVxuXG4gICAgcmV0dXJuIG51bGw7IC8vIG5vIGxpbmsuXG4gIH1cblxuICBmdW5jdGlvbiBjbGVhcigpIHtcbiAgICBlbnRlck1vZGlmaWNhdGlvbigpO1xuICAgIGZvckVhY2hOb2RlKGZ1bmN0aW9uKG5vZGUpIHtcbiAgICAgIHJlbW92ZU5vZGUobm9kZS5pZCk7XG4gICAgfSk7XG4gICAgZXhpdE1vZGlmaWNhdGlvbigpO1xuICB9XG5cbiAgZnVuY3Rpb24gZm9yRWFjaExpbmsoY2FsbGJhY2spIHtcbiAgICB2YXIgaSwgbGVuZ3RoO1xuICAgIGlmICh0eXBlb2YgY2FsbGJhY2sgPT09ICdmdW5jdGlvbicpIHtcbiAgICAgIGZvciAoaSA9IDAsIGxlbmd0aCA9IGxpbmtzLmxlbmd0aDsgaSA8IGxlbmd0aDsgKytpKSB7XG4gICAgICAgIGNhbGxiYWNrKGxpbmtzW2ldKTtcbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICBmdW5jdGlvbiBmb3JFYWNoTGlua2VkTm9kZShub2RlSWQsIGNhbGxiYWNrLCBvcmllbnRlZCkge1xuICAgIHZhciBub2RlID0gZ2V0Tm9kZShub2RlSWQpO1xuXG4gICAgaWYgKG5vZGUgJiYgbm9kZS5saW5rcyAmJiB0eXBlb2YgY2FsbGJhY2sgPT09ICdmdW5jdGlvbicpIHtcbiAgICAgIGlmIChvcmllbnRlZCkge1xuICAgICAgICByZXR1cm4gZm9yRWFjaE9yaWVudGVkTGluayhub2RlLmxpbmtzLCBub2RlSWQsIGNhbGxiYWNrKTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHJldHVybiBmb3JFYWNoTm9uT3JpZW50ZWRMaW5rKG5vZGUubGlua3MsIG5vZGVJZCwgY2FsbGJhY2spO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIGZvckVhY2hOb25PcmllbnRlZExpbmsobGlua3MsIG5vZGVJZCwgY2FsbGJhY2spIHtcbiAgICB2YXIgcXVpdEZhc3Q7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsaW5rcy5sZW5ndGg7ICsraSkge1xuICAgICAgdmFyIGxpbmsgPSBsaW5rc1tpXTtcbiAgICAgIHZhciBsaW5rZWROb2RlSWQgPSBsaW5rLmZyb21JZCA9PT0gbm9kZUlkID8gbGluay50b0lkIDogbGluay5mcm9tSWQ7XG5cbiAgICAgIHF1aXRGYXN0ID0gY2FsbGJhY2sobm9kZXNbbGlua2VkTm9kZUlkXSwgbGluayk7XG4gICAgICBpZiAocXVpdEZhc3QpIHtcbiAgICAgICAgcmV0dXJuIHRydWU7IC8vIENsaWVudCBkb2VzIG5vdCBuZWVkIG1vcmUgaXRlcmF0aW9ucy4gQnJlYWsgbm93LlxuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIGZvckVhY2hPcmllbnRlZExpbmsobGlua3MsIG5vZGVJZCwgY2FsbGJhY2spIHtcbiAgICB2YXIgcXVpdEZhc3Q7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsaW5rcy5sZW5ndGg7ICsraSkge1xuICAgICAgdmFyIGxpbmsgPSBsaW5rc1tpXTtcbiAgICAgIGlmIChsaW5rLmZyb21JZCA9PT0gbm9kZUlkKSB7XG4gICAgICAgIHF1aXRGYXN0ID0gY2FsbGJhY2sobm9kZXNbbGluay50b0lkXSwgbGluayk7XG4gICAgICAgIGlmIChxdWl0RmFzdCkge1xuICAgICAgICAgIHJldHVybiB0cnVlOyAvLyBDbGllbnQgZG9lcyBub3QgbmVlZCBtb3JlIGl0ZXJhdGlvbnMuIEJyZWFrIG5vdy5cbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIC8vIHdlIHdpbGwgbm90IGZpcmUgYW55dGhpbmcgdW50aWwgdXNlcnMgb2YgdGhpcyBsaWJyYXJ5IGV4cGxpY2l0bHkgY2FsbCBgb24oKWBcbiAgLy8gbWV0aG9kLlxuICBmdW5jdGlvbiBub29wKCkge31cblxuICAvLyBFbnRlciwgRXhpdCBtb2RpZmljYXRpb24gYWxsb3dzIGJ1bGsgZ3JhcGggdXBkYXRlcyB3aXRob3V0IGZpcmluZyBldmVudHMuXG4gIGZ1bmN0aW9uIGVudGVyTW9kaWZpY2F0aW9uUmVhbCgpIHtcbiAgICBzdXNwZW5kRXZlbnRzICs9IDE7XG4gIH1cblxuICBmdW5jdGlvbiBleGl0TW9kaWZpY2F0aW9uUmVhbCgpIHtcbiAgICBzdXNwZW5kRXZlbnRzIC09IDE7XG4gICAgaWYgKHN1c3BlbmRFdmVudHMgPT09IDAgJiYgY2hhbmdlcy5sZW5ndGggPiAwKSB7XG4gICAgICBncmFwaFBhcnQuZmlyZSgnY2hhbmdlZCcsIGNoYW5nZXMpO1xuICAgICAgY2hhbmdlcy5sZW5ndGggPSAwO1xuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIGNyZWF0ZU5vZGVJdGVyYXRvcigpIHtcbiAgICAvLyBPYmplY3Qua2V5cyBpdGVyYXRvciBpcyAxLjN4IGZhc3RlciB0aGFuIGBmb3IgaW5gIGxvb3AuXG4gICAgLy8gU2VlIGBodHRwczovL2dpdGh1Yi5jb20vYW52YWthL25ncmFwaC5ncmFwaC90cmVlL2JlbmNoLWZvci1pbi12cy1vYmota2V5c2BcbiAgICAvLyBicmFuY2ggZm9yIHBlcmYgdGVzdFxuICAgIHJldHVybiBPYmplY3Qua2V5cyA/IG9iamVjdEtleXNJdGVyYXRvciA6IGZvckluSXRlcmF0b3I7XG4gIH1cblxuICBmdW5jdGlvbiBvYmplY3RLZXlzSXRlcmF0b3IoY2FsbGJhY2spIHtcbiAgICBpZiAodHlwZW9mIGNhbGxiYWNrICE9PSAnZnVuY3Rpb24nKSB7XG4gICAgICByZXR1cm47XG4gICAgfVxuXG4gICAgdmFyIGtleXMgPSBPYmplY3Qua2V5cyhub2Rlcyk7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBrZXlzLmxlbmd0aDsgKytpKSB7XG4gICAgICBpZiAoY2FsbGJhY2sobm9kZXNba2V5c1tpXV0pKSB7XG4gICAgICAgIHJldHVybiB0cnVlOyAvLyBjbGllbnQgZG9lc24ndCB3YW50IHRvIHByb2NlZWQuIFJldHVybi5cbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICBmdW5jdGlvbiBmb3JJbkl0ZXJhdG9yKGNhbGxiYWNrKSB7XG4gICAgaWYgKHR5cGVvZiBjYWxsYmFjayAhPT0gJ2Z1bmN0aW9uJykge1xuICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICB2YXIgbm9kZTtcblxuICAgIGZvciAobm9kZSBpbiBub2Rlcykge1xuICAgICAgaWYgKGNhbGxiYWNrKG5vZGVzW25vZGVdKSkge1xuICAgICAgICByZXR1cm4gdHJ1ZTsgLy8gY2xpZW50IGRvZXNuJ3Qgd2FudCB0byBwcm9jZWVkLiBSZXR1cm4uXG4gICAgICB9XG4gICAgfVxuICB9XG59XG5cbi8vIG5lZWQgdGhpcyBmb3Igb2xkIGJyb3dzZXJzLiBTaG91bGQgdGhpcyBiZSBhIHNlcGFyYXRlIG1vZHVsZT9cbmZ1bmN0aW9uIGluZGV4T2ZFbGVtZW50SW5BcnJheShlbGVtZW50LCBhcnJheSkge1xuICBpZiAoIWFycmF5KSByZXR1cm4gLTE7XG5cbiAgaWYgKGFycmF5LmluZGV4T2YpIHtcbiAgICByZXR1cm4gYXJyYXkuaW5kZXhPZihlbGVtZW50KTtcbiAgfVxuXG4gIHZhciBsZW4gPSBhcnJheS5sZW5ndGgsXG4gICAgaTtcblxuICBmb3IgKGkgPSAwOyBpIDwgbGVuOyBpICs9IDEpIHtcbiAgICBpZiAoYXJyYXlbaV0gPT09IGVsZW1lbnQpIHtcbiAgICAgIHJldHVybiBpO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiAtMTtcbn1cblxuLyoqXG4gKiBJbnRlcm5hbCBzdHJ1Y3R1cmUgdG8gcmVwcmVzZW50IG5vZGU7XG4gKi9cbmZ1bmN0aW9uIE5vZGUoaWQsIGRhdGEpIHtcbiAgdGhpcy5pZCA9IGlkO1xuICB0aGlzLmxpbmtzID0gbnVsbDtcbiAgdGhpcy5kYXRhID0gZGF0YTtcbn1cblxuZnVuY3Rpb24gYWRkTGlua1RvTm9kZShub2RlLCBsaW5rKSB7XG4gIGlmIChub2RlLmxpbmtzKSB7XG4gICAgbm9kZS5saW5rcy5wdXNoKGxpbmspO1xuICB9IGVsc2Uge1xuICAgIG5vZGUubGlua3MgPSBbbGlua107XG4gIH1cbn1cblxuLyoqXG4gKiBJbnRlcm5hbCBzdHJ1Y3R1cmUgdG8gcmVwcmVzZW50IGxpbmtzO1xuICovXG5mdW5jdGlvbiBMaW5rKGZyb21JZCwgdG9JZCwgZGF0YSwgaWQpIHtcbiAgdGhpcy5mcm9tSWQgPSBmcm9tSWQ7XG4gIHRoaXMudG9JZCA9IHRvSWQ7XG4gIHRoaXMuZGF0YSA9IGRhdGE7XG4gIHRoaXMuaWQgPSBpZDtcbn1cblxuZnVuY3Rpb24gaGFzaENvZGUoc3RyKSB7XG4gIHZhciBoYXNoID0gMCwgaSwgY2hyLCBsZW47XG4gIGlmIChzdHIubGVuZ3RoID09IDApIHJldHVybiBoYXNoO1xuICBmb3IgKGkgPSAwLCBsZW4gPSBzdHIubGVuZ3RoOyBpIDwgbGVuOyBpKyspIHtcbiAgICBjaHIgICA9IHN0ci5jaGFyQ29kZUF0KGkpO1xuICAgIGhhc2ggID0gKChoYXNoIDw8IDUpIC0gaGFzaCkgKyBjaHI7XG4gICAgaGFzaCB8PSAwOyAvLyBDb252ZXJ0IHRvIDMyYml0IGludGVnZXJcbiAgfVxuICByZXR1cm4gaGFzaDtcbn1cblxuZnVuY3Rpb24gbWFrZUxpbmtJZChmcm9tSWQsIHRvSWQpIHtcbiAgcmV0dXJuIGZyb21JZC50b1N0cmluZygpICsgJ/CfkYkgJyArIHRvSWQudG9TdHJpbmcoKTtcbn1cbiIsIm1vZHVsZS5leHBvcnRzID0gbWVyZ2U7XG5cbi8qKlxuICogQXVnbWVudHMgYHRhcmdldGAgd2l0aCBwcm9wZXJ0aWVzIGluIGBvcHRpb25zYC4gRG9lcyBub3Qgb3ZlcnJpZGVcbiAqIHRhcmdldCdzIHByb3BlcnRpZXMgaWYgdGhleSBhcmUgZGVmaW5lZCBhbmQgbWF0Y2hlcyBleHBlY3RlZCB0eXBlIGluIFxuICogb3B0aW9uc1xuICpcbiAqIEByZXR1cm5zIHtPYmplY3R9IG1lcmdlZCBvYmplY3RcbiAqL1xuZnVuY3Rpb24gbWVyZ2UodGFyZ2V0LCBvcHRpb25zKSB7XG4gIHZhciBrZXk7XG4gIGlmICghdGFyZ2V0KSB7IHRhcmdldCA9IHt9OyB9XG4gIGlmIChvcHRpb25zKSB7XG4gICAgZm9yIChrZXkgaW4gb3B0aW9ucykge1xuICAgICAgaWYgKG9wdGlvbnMuaGFzT3duUHJvcGVydHkoa2V5KSkge1xuICAgICAgICB2YXIgdGFyZ2V0SGFzSXQgPSB0YXJnZXQuaGFzT3duUHJvcGVydHkoa2V5KSxcbiAgICAgICAgICAgIG9wdGlvbnNWYWx1ZVR5cGUgPSB0eXBlb2Ygb3B0aW9uc1trZXldLFxuICAgICAgICAgICAgc2hvdWxkUmVwbGFjZSA9ICF0YXJnZXRIYXNJdCB8fCAodHlwZW9mIHRhcmdldFtrZXldICE9PSBvcHRpb25zVmFsdWVUeXBlKTtcblxuICAgICAgICBpZiAoc2hvdWxkUmVwbGFjZSkge1xuICAgICAgICAgIHRhcmdldFtrZXldID0gb3B0aW9uc1trZXldO1xuICAgICAgICB9IGVsc2UgaWYgKG9wdGlvbnNWYWx1ZVR5cGUgPT09ICdvYmplY3QnKSB7XG4gICAgICAgICAgLy8gZ28gZGVlcCwgZG9uJ3QgY2FyZSBhYm91dCBsb29wcyBoZXJlLCB3ZSBhcmUgc2ltcGxlIEFQSSE6XG4gICAgICAgICAgdGFyZ2V0W2tleV0gPSBtZXJnZSh0YXJnZXRba2V5XSwgb3B0aW9uc1trZXldKTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIHJldHVybiB0YXJnZXQ7XG59XG4iLCJtb2R1bGUuZXhwb3J0cyA9IHtcbiAgQm9keTogQm9keSxcbiAgVmVjdG9yMmQ6IFZlY3RvcjJkLFxuICBCb2R5M2Q6IEJvZHkzZCxcbiAgVmVjdG9yM2Q6IFZlY3RvcjNkXG59O1xuXG5mdW5jdGlvbiBCb2R5KHgsIHkpIHtcbiAgdGhpcy5wb3MgPSBuZXcgVmVjdG9yMmQoeCwgeSk7XG4gIHRoaXMucHJldlBvcyA9IG5ldyBWZWN0b3IyZCh4LCB5KTtcbiAgdGhpcy5mb3JjZSA9IG5ldyBWZWN0b3IyZCgpO1xuICB0aGlzLnZlbG9jaXR5ID0gbmV3IFZlY3RvcjJkKCk7XG4gIHRoaXMubWFzcyA9IDE7XG59XG5cbkJvZHkucHJvdG90eXBlLnNldFBvc2l0aW9uID0gZnVuY3Rpb24gKHgsIHkpIHtcbiAgdGhpcy5wcmV2UG9zLnggPSB0aGlzLnBvcy54ID0geDtcbiAgdGhpcy5wcmV2UG9zLnkgPSB0aGlzLnBvcy55ID0geTtcbn07XG5cbmZ1bmN0aW9uIFZlY3RvcjJkKHgsIHkpIHtcbiAgaWYgKHggJiYgdHlwZW9mIHggIT09ICdudW1iZXInKSB7XG4gICAgLy8gY291bGQgYmUgYW5vdGhlciB2ZWN0b3JcbiAgICB0aGlzLnggPSB0eXBlb2YgeC54ID09PSAnbnVtYmVyJyA/IHgueCA6IDA7XG4gICAgdGhpcy55ID0gdHlwZW9mIHgueSA9PT0gJ251bWJlcicgPyB4LnkgOiAwO1xuICB9IGVsc2Uge1xuICAgIHRoaXMueCA9IHR5cGVvZiB4ID09PSAnbnVtYmVyJyA/IHggOiAwO1xuICAgIHRoaXMueSA9IHR5cGVvZiB5ID09PSAnbnVtYmVyJyA/IHkgOiAwO1xuICB9XG59XG5cblZlY3RvcjJkLnByb3RvdHlwZS5yZXNldCA9IGZ1bmN0aW9uICgpIHtcbiAgdGhpcy54ID0gdGhpcy55ID0gMDtcbn07XG5cbmZ1bmN0aW9uIEJvZHkzZCh4LCB5LCB6KSB7XG4gIHRoaXMucG9zID0gbmV3IFZlY3RvcjNkKHgsIHksIHopO1xuICB0aGlzLnByZXZQb3MgPSBuZXcgVmVjdG9yM2QoeCwgeSwgeik7XG4gIHRoaXMuZm9yY2UgPSBuZXcgVmVjdG9yM2QoKTtcbiAgdGhpcy52ZWxvY2l0eSA9IG5ldyBWZWN0b3IzZCgpO1xuICB0aGlzLm1hc3MgPSAxO1xufVxuXG5Cb2R5M2QucHJvdG90eXBlLnNldFBvc2l0aW9uID0gZnVuY3Rpb24gKHgsIHksIHopIHtcbiAgdGhpcy5wcmV2UG9zLnggPSB0aGlzLnBvcy54ID0geDtcbiAgdGhpcy5wcmV2UG9zLnkgPSB0aGlzLnBvcy55ID0geTtcbiAgdGhpcy5wcmV2UG9zLnogPSB0aGlzLnBvcy56ID0gejtcbn07XG5cbmZ1bmN0aW9uIFZlY3RvcjNkKHgsIHksIHopIHtcbiAgaWYgKHggJiYgdHlwZW9mIHggIT09ICdudW1iZXInKSB7XG4gICAgLy8gY291bGQgYmUgYW5vdGhlciB2ZWN0b3JcbiAgICB0aGlzLnggPSB0eXBlb2YgeC54ID09PSAnbnVtYmVyJyA/IHgueCA6IDA7XG4gICAgdGhpcy55ID0gdHlwZW9mIHgueSA9PT0gJ251bWJlcicgPyB4LnkgOiAwO1xuICAgIHRoaXMueiA9IHR5cGVvZiB4LnogPT09ICdudW1iZXInID8geC56IDogMDtcbiAgfSBlbHNlIHtcbiAgICB0aGlzLnggPSB0eXBlb2YgeCA9PT0gJ251bWJlcicgPyB4IDogMDtcbiAgICB0aGlzLnkgPSB0eXBlb2YgeSA9PT0gJ251bWJlcicgPyB5IDogMDtcbiAgICB0aGlzLnogPSB0eXBlb2YgeiA9PT0gJ251bWJlcicgPyB6IDogMDtcbiAgfVxufTtcblxuVmVjdG9yM2QucHJvdG90eXBlLnJlc2V0ID0gZnVuY3Rpb24gKCkge1xuICB0aGlzLnggPSB0aGlzLnkgPSB0aGlzLnogPSAwO1xufTtcbiIsIi8qKlxuICogTWFuYWdlcyBhIHNpbXVsYXRpb24gb2YgcGh5c2ljYWwgZm9yY2VzIGFjdGluZyBvbiBib2RpZXMgYW5kIHNwcmluZ3MuXG4gKi9cbm1vZHVsZS5leHBvcnRzID0gcGh5c2ljc1NpbXVsYXRvcjtcblxuZnVuY3Rpb24gcGh5c2ljc1NpbXVsYXRvcihzZXR0aW5ncykge1xuICB2YXIgU3ByaW5nID0gcmVxdWlyZSgnLi9saWIvc3ByaW5nJyk7XG4gIHZhciBleHBvc2UgPSByZXF1aXJlKCduZ3JhcGguZXhwb3NlJyk7XG4gIHZhciBtZXJnZSA9IHJlcXVpcmUoJ25ncmFwaC5tZXJnZScpO1xuXG4gIHNldHRpbmdzID0gbWVyZ2Uoc2V0dGluZ3MsIHtcbiAgICAgIC8qKlxuICAgICAgICogSWRlYWwgbGVuZ3RoIGZvciBsaW5rcyAoc3ByaW5ncyBpbiBwaHlzaWNhbCBtb2RlbCkuXG4gICAgICAgKi9cbiAgICAgIHNwcmluZ0xlbmd0aDogMzAsXG5cbiAgICAgIC8qKlxuICAgICAgICogSG9vaydzIGxhdyBjb2VmZmljaWVudC4gMSAtIHNvbGlkIHNwcmluZy5cbiAgICAgICAqL1xuICAgICAgc3ByaW5nQ29lZmY6IDAuMDAwOCxcblxuICAgICAgLyoqXG4gICAgICAgKiBDb3Vsb21iJ3MgbGF3IGNvZWZmaWNpZW50LiBJdCdzIHVzZWQgdG8gcmVwZWwgbm9kZXMgdGh1cyBzaG91bGQgYmUgbmVnYXRpdmVcbiAgICAgICAqIGlmIHlvdSBtYWtlIGl0IHBvc2l0aXZlIG5vZGVzIHN0YXJ0IGF0dHJhY3QgZWFjaCBvdGhlciA6KS5cbiAgICAgICAqL1xuICAgICAgZ3Jhdml0eTogLTEuMixcblxuICAgICAgLyoqXG4gICAgICAgKiBUaGV0YSBjb2VmZmljaWVudCBmcm9tIEJhcm5lcyBIdXQgc2ltdWxhdGlvbi4gUmFuZ2VkIGJldHdlZW4gKDAsIDEpLlxuICAgICAgICogVGhlIGNsb3NlciBpdCdzIHRvIDEgdGhlIG1vcmUgbm9kZXMgYWxnb3JpdGhtIHdpbGwgaGF2ZSB0byBnbyB0aHJvdWdoLlxuICAgICAgICogU2V0dGluZyBpdCB0byBvbmUgbWFrZXMgQmFybmVzIEh1dCBzaW11bGF0aW9uIG5vIGRpZmZlcmVudCBmcm9tXG4gICAgICAgKiBicnV0ZS1mb3JjZSBmb3JjZXMgY2FsY3VsYXRpb24gKGVhY2ggbm9kZSBpcyBjb25zaWRlcmVkKS5cbiAgICAgICAqL1xuICAgICAgdGhldGE6IDAuOCxcblxuICAgICAgLyoqXG4gICAgICAgKiBEcmFnIGZvcmNlIGNvZWZmaWNpZW50LiBVc2VkIHRvIHNsb3cgZG93biBzeXN0ZW0sIHRodXMgc2hvdWxkIGJlIGxlc3MgdGhhbiAxLlxuICAgICAgICogVGhlIGNsb3NlciBpdCBpcyB0byAwIHRoZSBsZXNzIHRpZ2h0IHN5c3RlbSB3aWxsIGJlLlxuICAgICAgICovXG4gICAgICBkcmFnQ29lZmY6IDAuMDIsXG5cbiAgICAgIC8qKlxuICAgICAgICogRGVmYXVsdCB0aW1lIHN0ZXAgKGR0KSBmb3IgZm9yY2VzIGludGVncmF0aW9uXG4gICAgICAgKi9cbiAgICAgIHRpbWVTdGVwIDogMjAsXG5cbiAgICAgIC8qKlxuICAgICAgICAqIE1heGltdW0gbW92ZW1lbnQgb2YgdGhlIHN5c3RlbSB3aGljaCBjYW4gYmUgY29uc2lkZXJlZCBhcyBzdGFiaWxpemVkXG4gICAgICAgICovXG4gICAgICBzdGFibGVUaHJlc2hvbGQ6IDAuMDA5XG4gIH0pO1xuXG4gIC8vIFdlIGFsbG93IGNsaWVudHMgdG8gb3ZlcnJpZGUgYmFzaWMgZmFjdG9yeSBtZXRob2RzOlxuICB2YXIgY3JlYXRlUXVhZFRyZWUgPSBzZXR0aW5ncy5jcmVhdGVRdWFkVHJlZSB8fCByZXF1aXJlKCduZ3JhcGgucXVhZHRyZWViaCcpO1xuICB2YXIgY3JlYXRlQm91bmRzID0gc2V0dGluZ3MuY3JlYXRlQm91bmRzIHx8IHJlcXVpcmUoJy4vbGliL2JvdW5kcycpO1xuICB2YXIgY3JlYXRlRHJhZ0ZvcmNlID0gc2V0dGluZ3MuY3JlYXRlRHJhZ0ZvcmNlIHx8IHJlcXVpcmUoJy4vbGliL2RyYWdGb3JjZScpO1xuICB2YXIgY3JlYXRlU3ByaW5nRm9yY2UgPSBzZXR0aW5ncy5jcmVhdGVTcHJpbmdGb3JjZSB8fCByZXF1aXJlKCcuL2xpYi9zcHJpbmdGb3JjZScpO1xuICB2YXIgaW50ZWdyYXRlID0gc2V0dGluZ3MuaW50ZWdyYXRvciB8fCByZXF1aXJlKCcuL2xpYi9ldWxlckludGVncmF0b3InKTtcbiAgdmFyIGNyZWF0ZUJvZHkgPSBzZXR0aW5ncy5jcmVhdGVCb2R5IHx8IHJlcXVpcmUoJy4vbGliL2NyZWF0ZUJvZHknKTtcblxuICB2YXIgYm9kaWVzID0gW10sIC8vIEJvZGllcyBpbiB0aGlzIHNpbXVsYXRpb24uXG4gICAgICBzcHJpbmdzID0gW10sIC8vIFNwcmluZ3MgaW4gdGhpcyBzaW11bGF0aW9uLlxuICAgICAgcXVhZFRyZWUgPSAgY3JlYXRlUXVhZFRyZWUoc2V0dGluZ3MpLFxuICAgICAgYm91bmRzID0gY3JlYXRlQm91bmRzKGJvZGllcywgc2V0dGluZ3MpLFxuICAgICAgc3ByaW5nRm9yY2UgPSBjcmVhdGVTcHJpbmdGb3JjZShzZXR0aW5ncyksXG4gICAgICBkcmFnRm9yY2UgPSBjcmVhdGVEcmFnRm9yY2Uoc2V0dGluZ3MpO1xuXG4gIHZhciBwdWJsaWNBcGkgPSB7XG4gICAgLyoqXG4gICAgICogQXJyYXkgb2YgYm9kaWVzLCByZWdpc3RlcmVkIHdpdGggY3VycmVudCBzaW11bGF0b3JcbiAgICAgKlxuICAgICAqIE5vdGU6IFRvIGFkZCBuZXcgYm9keSwgdXNlIGFkZEJvZHkoKSBtZXRob2QuIFRoaXMgcHJvcGVydHkgaXMgb25seVxuICAgICAqIGV4cG9zZWQgZm9yIHRlc3RpbmcvcGVyZm9ybWFuY2UgcHVycG9zZXMuXG4gICAgICovXG4gICAgYm9kaWVzOiBib2RpZXMsXG5cbiAgICAvKipcbiAgICAgKiBBcnJheSBvZiBzcHJpbmdzLCByZWdpc3RlcmVkIHdpdGggY3VycmVudCBzaW11bGF0b3JcbiAgICAgKlxuICAgICAqIE5vdGU6IFRvIGFkZCBuZXcgc3ByaW5nLCB1c2UgYWRkU3ByaW5nKCkgbWV0aG9kLiBUaGlzIHByb3BlcnR5IGlzIG9ubHlcbiAgICAgKiBleHBvc2VkIGZvciB0ZXN0aW5nL3BlcmZvcm1hbmNlIHB1cnBvc2VzLlxuICAgICAqL1xuICAgIHNwcmluZ3M6IHNwcmluZ3MsXG5cbiAgICAvKipcbiAgICAgKiBSZXR1cm5zIHNldHRpbmdzIHdpdGggd2hpY2ggY3VycmVudCBzaW11bGF0b3Igd2FzIGluaXRpYWxpemVkXG4gICAgICovXG4gICAgc2V0dGluZ3M6IHNldHRpbmdzLFxuXG4gICAgLyoqXG4gICAgICogUGVyZm9ybXMgb25lIHN0ZXAgb2YgZm9yY2Ugc2ltdWxhdGlvbi5cbiAgICAgKlxuICAgICAqIEByZXR1cm5zIHtib29sZWFufSB0cnVlIGlmIHN5c3RlbSBpcyBjb25zaWRlcmVkIHN0YWJsZTsgRmFsc2Ugb3RoZXJ3aXNlLlxuICAgICAqL1xuICAgIHN0ZXA6IGZ1bmN0aW9uICgpIHtcbiAgICAgIGFjY3VtdWxhdGVGb3JjZXMoKTtcbiAgICAgIHZhciB0b3RhbE1vdmVtZW50ID0gaW50ZWdyYXRlKGJvZGllcywgc2V0dGluZ3MudGltZVN0ZXApO1xuXG4gICAgICBib3VuZHMudXBkYXRlKCk7XG5cbiAgICAgIHJldHVybiB0b3RhbE1vdmVtZW50IDwgc2V0dGluZ3Muc3RhYmxlVGhyZXNob2xkO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBBZGRzIGJvZHkgdG8gdGhlIHN5c3RlbVxuICAgICAqXG4gICAgICogQHBhcmFtIHtuZ3JhcGgucGh5c2ljcy5wcmltaXRpdmVzLkJvZHl9IGJvZHkgcGh5c2ljYWwgYm9keVxuICAgICAqXG4gICAgICogQHJldHVybnMge25ncmFwaC5waHlzaWNzLnByaW1pdGl2ZXMuQm9keX0gYWRkZWQgYm9keVxuICAgICAqL1xuICAgIGFkZEJvZHk6IGZ1bmN0aW9uIChib2R5KSB7XG4gICAgICBpZiAoIWJvZHkpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdCb2R5IGlzIHJlcXVpcmVkJyk7XG4gICAgICB9XG4gICAgICBib2RpZXMucHVzaChib2R5KTtcblxuICAgICAgcmV0dXJuIGJvZHk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEFkZHMgYm9keSB0byB0aGUgc3lzdGVtIGF0IGdpdmVuIHBvc2l0aW9uXG4gICAgICpcbiAgICAgKiBAcGFyYW0ge09iamVjdH0gcG9zIHBvc2l0aW9uIG9mIGEgYm9keVxuICAgICAqXG4gICAgICogQHJldHVybnMge25ncmFwaC5waHlzaWNzLnByaW1pdGl2ZXMuQm9keX0gYWRkZWQgYm9keVxuICAgICAqL1xuICAgIGFkZEJvZHlBdDogZnVuY3Rpb24gKHBvcykge1xuICAgICAgaWYgKCFwb3MpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdCb2R5IHBvc2l0aW9uIGlzIHJlcXVpcmVkJyk7XG4gICAgICB9XG4gICAgICB2YXIgYm9keSA9IGNyZWF0ZUJvZHkocG9zKTtcbiAgICAgIGJvZGllcy5wdXNoKGJvZHkpO1xuXG4gICAgICByZXR1cm4gYm9keTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogUmVtb3ZlcyBib2R5IGZyb20gdGhlIHN5c3RlbVxuICAgICAqXG4gICAgICogQHBhcmFtIHtuZ3JhcGgucGh5c2ljcy5wcmltaXRpdmVzLkJvZHl9IGJvZHkgdG8gcmVtb3ZlXG4gICAgICpcbiAgICAgKiBAcmV0dXJucyB7Qm9vbGVhbn0gdHJ1ZSBpZiBib2R5IGZvdW5kIGFuZCByZW1vdmVkLiBmYWxzeSBvdGhlcndpc2U7XG4gICAgICovXG4gICAgcmVtb3ZlQm9keTogZnVuY3Rpb24gKGJvZHkpIHtcbiAgICAgIGlmICghYm9keSkgeyByZXR1cm47IH1cblxuICAgICAgdmFyIGlkeCA9IGJvZGllcy5pbmRleE9mKGJvZHkpO1xuICAgICAgaWYgKGlkeCA8IDApIHsgcmV0dXJuOyB9XG5cbiAgICAgIGJvZGllcy5zcGxpY2UoaWR4LCAxKTtcbiAgICAgIGlmIChib2RpZXMubGVuZ3RoID09PSAwKSB7XG4gICAgICAgIGJvdW5kcy5yZXNldCgpO1xuICAgICAgfVxuICAgICAgcmV0dXJuIHRydWU7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEFkZHMgYSBzcHJpbmcgdG8gdGhpcyBzaW11bGF0aW9uLlxuICAgICAqXG4gICAgICogQHJldHVybnMge09iamVjdH0gLSBhIGhhbmRsZSBmb3IgYSBzcHJpbmcuIElmIHlvdSB3YW50IHRvIGxhdGVyIHJlbW92ZVxuICAgICAqIHNwcmluZyBwYXNzIGl0IHRvIHJlbW92ZVNwcmluZygpIG1ldGhvZC5cbiAgICAgKi9cbiAgICBhZGRTcHJpbmc6IGZ1bmN0aW9uIChib2R5MSwgYm9keTIsIHNwcmluZ0xlbmd0aCwgc3ByaW5nV2VpZ2h0LCBzcHJpbmdDb2VmZmljaWVudCkge1xuICAgICAgaWYgKCFib2R5MSB8fCAhYm9keTIpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdDYW5ub3QgYWRkIG51bGwgc3ByaW5nIHRvIGZvcmNlIHNpbXVsYXRvcicpO1xuICAgICAgfVxuXG4gICAgICBpZiAodHlwZW9mIHNwcmluZ0xlbmd0aCAhPT0gJ251bWJlcicpIHtcbiAgICAgICAgc3ByaW5nTGVuZ3RoID0gLTE7IC8vIGFzc3VtZSBnbG9iYWwgY29uZmlndXJhdGlvblxuICAgICAgfVxuXG4gICAgICB2YXIgc3ByaW5nID0gbmV3IFNwcmluZyhib2R5MSwgYm9keTIsIHNwcmluZ0xlbmd0aCwgc3ByaW5nQ29lZmZpY2llbnQgPj0gMCA/IHNwcmluZ0NvZWZmaWNpZW50IDogLTEsIHNwcmluZ1dlaWdodCk7XG4gICAgICBzcHJpbmdzLnB1c2goc3ByaW5nKTtcblxuICAgICAgLy8gVE9ETzogY291bGQgbWFyayBzaW11bGF0b3IgYXMgZGlydHkuXG4gICAgICByZXR1cm4gc3ByaW5nO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBSZW1vdmVzIHNwcmluZyBmcm9tIHRoZSBzeXN0ZW1cbiAgICAgKlxuICAgICAqIEBwYXJhbSB7T2JqZWN0fSBzcHJpbmcgdG8gcmVtb3ZlLiBTcHJpbmcgaXMgYW4gb2JqZWN0IHJldHVybmVkIGJ5IGFkZFNwcmluZ1xuICAgICAqXG4gICAgICogQHJldHVybnMge0Jvb2xlYW59IHRydWUgaWYgc3ByaW5nIGZvdW5kIGFuZCByZW1vdmVkLiBmYWxzeSBvdGhlcndpc2U7XG4gICAgICovXG4gICAgcmVtb3ZlU3ByaW5nOiBmdW5jdGlvbiAoc3ByaW5nKSB7XG4gICAgICBpZiAoIXNwcmluZykgeyByZXR1cm47IH1cbiAgICAgIHZhciBpZHggPSBzcHJpbmdzLmluZGV4T2Yoc3ByaW5nKTtcbiAgICAgIGlmIChpZHggPiAtMSkge1xuICAgICAgICBzcHJpbmdzLnNwbGljZShpZHgsIDEpO1xuICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgIH1cbiAgICB9LFxuXG4gICAgZ2V0QmVzdE5ld0JvZHlQb3NpdGlvbjogZnVuY3Rpb24gKG5laWdoYm9ycykge1xuICAgICAgcmV0dXJuIGJvdW5kcy5nZXRCZXN0TmV3UG9zaXRpb24obmVpZ2hib3JzKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogUmV0dXJucyBib3VuZGluZyBib3ggd2hpY2ggY292ZXJzIGFsbCBib2RpZXNcbiAgICAgKi9cbiAgICBnZXRCQm94OiBmdW5jdGlvbiAoKSB7XG4gICAgICByZXR1cm4gYm91bmRzLmJveDtcbiAgICB9LFxuXG4gICAgZ3Jhdml0eTogZnVuY3Rpb24gKHZhbHVlKSB7XG4gICAgICBpZiAodmFsdWUgIT09IHVuZGVmaW5lZCkge1xuICAgICAgICBzZXR0aW5ncy5ncmF2aXR5ID0gdmFsdWU7XG4gICAgICAgIHF1YWRUcmVlLm9wdGlvbnMoe2dyYXZpdHk6IHZhbHVlfSk7XG4gICAgICAgIHJldHVybiB0aGlzO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0dXJuIHNldHRpbmdzLmdyYXZpdHk7XG4gICAgICB9XG4gICAgfSxcblxuICAgIHRoZXRhOiBmdW5jdGlvbiAodmFsdWUpIHtcbiAgICAgIGlmICh2YWx1ZSAhPT0gdW5kZWZpbmVkKSB7XG4gICAgICAgIHNldHRpbmdzLnRoZXRhID0gdmFsdWU7XG4gICAgICAgIHF1YWRUcmVlLm9wdGlvbnMoe3RoZXRhOiB2YWx1ZX0pO1xuICAgICAgICByZXR1cm4gdGhpcztcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHJldHVybiBzZXR0aW5ncy50aGV0YTtcbiAgICAgIH1cbiAgICB9XG4gIH07XG5cbiAgLy8gYWxsb3cgc2V0dGluZ3MgbW9kaWZpY2F0aW9uIHZpYSBwdWJsaWMgQVBJOlxuICBleHBvc2Uoc2V0dGluZ3MsIHB1YmxpY0FwaSk7XG5cbiAgcmV0dXJuIHB1YmxpY0FwaTtcblxuICBmdW5jdGlvbiBhY2N1bXVsYXRlRm9yY2VzKCkge1xuICAgIC8vIEFjY3VtdWxhdGUgZm9yY2VzIGFjdGluZyBvbiBib2RpZXMuXG4gICAgdmFyIGJvZHksXG4gICAgICAgIGkgPSBib2RpZXMubGVuZ3RoO1xuXG4gICAgaWYgKGkpIHtcbiAgICAgIC8vIG9ubHkgYWRkIGJvZGllcyBpZiB0aGVyZSB0aGUgYXJyYXkgaXMgbm90IGVtcHR5OlxuICAgICAgcXVhZFRyZWUuaW5zZXJ0Qm9kaWVzKGJvZGllcyk7IC8vIHBlcmZvcm1hbmNlOiBPKG4gKiBsb2cgbilcbiAgICAgIHdoaWxlIChpLS0pIHtcbiAgICAgICAgYm9keSA9IGJvZGllc1tpXTtcbiAgICAgICAgLy8gSWYgYm9keSBpcyBwaW5uZWQgdGhlcmUgaXMgbm8gcG9pbnQgdXBkYXRpbmcgaXRzIGZvcmNlcyAtIGl0IHNob3VsZFxuICAgICAgICAvLyBuZXZlciBtb3ZlOlxuICAgICAgICBpZiAoIWJvZHkuaXNQaW5uZWQpIHtcbiAgICAgICAgICBib2R5LmZvcmNlLnJlc2V0KCk7XG5cbiAgICAgICAgICBxdWFkVHJlZS51cGRhdGVCb2R5Rm9yY2UoYm9keSk7XG4gICAgICAgICAgZHJhZ0ZvcmNlLnVwZGF0ZShib2R5KTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cblxuICAgIGkgPSBzcHJpbmdzLmxlbmd0aDtcbiAgICB3aGlsZShpLS0pIHtcbiAgICAgIHNwcmluZ0ZvcmNlLnVwZGF0ZShzcHJpbmdzW2ldKTtcbiAgICB9XG4gIH1cbn07XG4iLCIvKipcbiAqIFBlcmZvcm1zIGZvcmNlcyBpbnRlZ3JhdGlvbiwgdXNpbmcgZ2l2ZW4gdGltZXN0ZXAuIFVzZXMgRXVsZXIgbWV0aG9kIHRvIHNvbHZlXG4gKiBkaWZmZXJlbnRpYWwgZXF1YXRpb24gKGh0dHA6Ly9lbi53aWtpcGVkaWEub3JnL3dpa2kvRXVsZXJfbWV0aG9kICkuXG4gKlxuICogQHJldHVybnMge051bWJlcn0gc3F1YXJlZCBkaXN0YW5jZSBvZiB0b3RhbCBwb3NpdGlvbiB1cGRhdGVzLlxuICovXG5cbm1vZHVsZS5leHBvcnRzID0gaW50ZWdyYXRlO1xuXG5mdW5jdGlvbiBpbnRlZ3JhdGUoYm9kaWVzLCB0aW1lU3RlcCkge1xuICB2YXIgZHggPSAwLCB0eCA9IDAsXG4gICAgICBkeSA9IDAsIHR5ID0gMCxcbiAgICAgIGksXG4gICAgICBtYXggPSBib2RpZXMubGVuZ3RoO1xuXG4gIGZvciAoaSA9IDA7IGkgPCBtYXg7ICsraSkge1xuICAgIHZhciBib2R5ID0gYm9kaWVzW2ldLFxuICAgICAgICBjb2VmZiA9IHRpbWVTdGVwIC8gYm9keS5tYXNzO1xuXG4gICAgYm9keS52ZWxvY2l0eS54ICs9IGNvZWZmICogYm9keS5mb3JjZS54O1xuICAgIGJvZHkudmVsb2NpdHkueSArPSBjb2VmZiAqIGJvZHkuZm9yY2UueTtcbiAgICB2YXIgdnggPSBib2R5LnZlbG9jaXR5LngsXG4gICAgICAgIHZ5ID0gYm9keS52ZWxvY2l0eS55LFxuICAgICAgICB2ID0gTWF0aC5zcXJ0KHZ4ICogdnggKyB2eSAqIHZ5KTtcblxuICAgIGlmICh2ID4gMSkge1xuICAgICAgYm9keS52ZWxvY2l0eS54ID0gdnggLyB2O1xuICAgICAgYm9keS52ZWxvY2l0eS55ID0gdnkgLyB2O1xuICAgIH1cblxuICAgIGR4ID0gdGltZVN0ZXAgKiBib2R5LnZlbG9jaXR5Lng7XG4gICAgZHkgPSB0aW1lU3RlcCAqIGJvZHkudmVsb2NpdHkueTtcblxuICAgIGJvZHkucG9zLnggKz0gZHg7XG4gICAgYm9keS5wb3MueSArPSBkeTtcblxuICAgIHR4ICs9IE1hdGguYWJzKGR4KTsgdHkgKz0gTWF0aC5hYnMoZHkpO1xuICB9XG5cbiAgcmV0dXJuICh0eCAqIHR4ICsgdHkgKiB0eSkvYm9kaWVzLmxlbmd0aDtcbn1cbiIsIi8qKlxuICogVGhpcyBpcyBCYXJuZXMgSHV0IHNpbXVsYXRpb24gYWxnb3JpdGhtIGZvciAyZCBjYXNlLiBJbXBsZW1lbnRhdGlvblxuICogaXMgaGlnaGx5IG9wdGltaXplZCAoYXZvaWRzIHJlY3VzaW9uIGFuZCBnYyBwcmVzc3VyZSlcbiAqXG4gKiBodHRwOi8vd3d3LmNzLnByaW5jZXRvbi5lZHUvY291cnNlcy9hcmNoaXZlL2ZhbGwwMy9jczEyNi9hc3NpZ25tZW50cy9iYXJuZXMtaHV0Lmh0bWxcbiAqL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uKG9wdGlvbnMpIHtcbiAgb3B0aW9ucyA9IG9wdGlvbnMgfHwge307XG4gIG9wdGlvbnMuZ3Jhdml0eSA9IHR5cGVvZiBvcHRpb25zLmdyYXZpdHkgPT09ICdudW1iZXInID8gb3B0aW9ucy5ncmF2aXR5IDogLTE7XG4gIG9wdGlvbnMudGhldGEgPSB0eXBlb2Ygb3B0aW9ucy50aGV0YSA9PT0gJ251bWJlcicgPyBvcHRpb25zLnRoZXRhIDogMC44O1xuXG4gIC8vIHdlIHJlcXVpcmUgZGV0ZXJtaW5pc3RpYyByYW5kb21uZXNzIGhlcmVcbiAgdmFyIHJhbmRvbSA9IHJlcXVpcmUoJ25ncmFwaC5yYW5kb20nKS5yYW5kb20oMTk4NCksXG4gICAgTm9kZSA9IHJlcXVpcmUoJy4vbm9kZScpLFxuICAgIEluc2VydFN0YWNrID0gcmVxdWlyZSgnLi9pbnNlcnRTdGFjaycpLFxuICAgIGlzU2FtZVBvc2l0aW9uID0gcmVxdWlyZSgnLi9pc1NhbWVQb3NpdGlvbicpO1xuXG4gIHZhciBncmF2aXR5ID0gb3B0aW9ucy5ncmF2aXR5LFxuICAgIHVwZGF0ZVF1ZXVlID0gW10sXG4gICAgaW5zZXJ0U3RhY2sgPSBuZXcgSW5zZXJ0U3RhY2soKSxcbiAgICB0aGV0YSA9IG9wdGlvbnMudGhldGEsXG5cbiAgICBub2Rlc0NhY2hlID0gW10sXG4gICAgY3VycmVudEluQ2FjaGUgPSAwLFxuICAgIG5ld05vZGUgPSBmdW5jdGlvbigpIHtcbiAgICAgIC8vIFRvIGF2b2lkIHByZXNzdXJlIG9uIEdDIHdlIHJldXNlIG5vZGVzLlxuICAgICAgdmFyIG5vZGUgPSBub2Rlc0NhY2hlW2N1cnJlbnRJbkNhY2hlXTtcbiAgICAgIGlmIChub2RlKSB7XG4gICAgICAgIG5vZGUucXVhZDAgPSBudWxsO1xuICAgICAgICBub2RlLnF1YWQxID0gbnVsbDtcbiAgICAgICAgbm9kZS5xdWFkMiA9IG51bGw7XG4gICAgICAgIG5vZGUucXVhZDMgPSBudWxsO1xuICAgICAgICBub2RlLmJvZHkgPSBudWxsO1xuICAgICAgICBub2RlLm1hc3MgPSBub2RlLm1hc3NYID0gbm9kZS5tYXNzWSA9IDA7XG4gICAgICAgIG5vZGUubGVmdCA9IG5vZGUucmlnaHQgPSBub2RlLnRvcCA9IG5vZGUuYm90dG9tID0gMDtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIG5vZGUgPSBuZXcgTm9kZSgpO1xuICAgICAgICBub2Rlc0NhY2hlW2N1cnJlbnRJbkNhY2hlXSA9IG5vZGU7XG4gICAgICB9XG5cbiAgICAgICsrY3VycmVudEluQ2FjaGU7XG4gICAgICByZXR1cm4gbm9kZTtcbiAgICB9LFxuXG4gICAgcm9vdCA9IG5ld05vZGUoKSxcblxuICAgIC8vIEluc2VydHMgYm9keSB0byB0aGUgdHJlZVxuICAgIGluc2VydCA9IGZ1bmN0aW9uKG5ld0JvZHkpIHtcbiAgICAgIGluc2VydFN0YWNrLnJlc2V0KCk7XG4gICAgICBpbnNlcnRTdGFjay5wdXNoKHJvb3QsIG5ld0JvZHkpO1xuXG4gICAgICB3aGlsZSAoIWluc2VydFN0YWNrLmlzRW1wdHkoKSkge1xuICAgICAgICB2YXIgc3RhY2tJdGVtID0gaW5zZXJ0U3RhY2sucG9wKCksXG4gICAgICAgICAgbm9kZSA9IHN0YWNrSXRlbS5ub2RlLFxuICAgICAgICAgIGJvZHkgPSBzdGFja0l0ZW0uYm9keTtcblxuICAgICAgICBpZiAoIW5vZGUuYm9keSkge1xuICAgICAgICAgIC8vIFRoaXMgaXMgaW50ZXJuYWwgbm9kZS4gVXBkYXRlIHRoZSB0b3RhbCBtYXNzIG9mIHRoZSBub2RlIGFuZCBjZW50ZXItb2YtbWFzcy5cbiAgICAgICAgICB2YXIgeCA9IGJvZHkucG9zLng7XG4gICAgICAgICAgdmFyIHkgPSBib2R5LnBvcy55O1xuICAgICAgICAgIG5vZGUubWFzcyA9IG5vZGUubWFzcyArIGJvZHkubWFzcztcbiAgICAgICAgICBub2RlLm1hc3NYID0gbm9kZS5tYXNzWCArIGJvZHkubWFzcyAqIHg7XG4gICAgICAgICAgbm9kZS5tYXNzWSA9IG5vZGUubWFzc1kgKyBib2R5Lm1hc3MgKiB5O1xuXG4gICAgICAgICAgLy8gUmVjdXJzaXZlbHkgaW5zZXJ0IHRoZSBib2R5IGluIHRoZSBhcHByb3ByaWF0ZSBxdWFkcmFudC5cbiAgICAgICAgICAvLyBCdXQgZmlyc3QgZmluZCB0aGUgYXBwcm9wcmlhdGUgcXVhZHJhbnQuXG4gICAgICAgICAgdmFyIHF1YWRJZHggPSAwLCAvLyBBc3N1bWUgd2UgYXJlIGluIHRoZSAwJ3MgcXVhZC5cbiAgICAgICAgICAgIGxlZnQgPSBub2RlLmxlZnQsXG4gICAgICAgICAgICByaWdodCA9IChub2RlLnJpZ2h0ICsgbGVmdCkgLyAyLFxuICAgICAgICAgICAgdG9wID0gbm9kZS50b3AsXG4gICAgICAgICAgICBib3R0b20gPSAobm9kZS5ib3R0b20gKyB0b3ApIC8gMjtcblxuICAgICAgICAgIGlmICh4ID4gcmlnaHQpIHsgLy8gc29tZXdoZXJlIGluIHRoZSBlYXN0ZXJuIHBhcnQuXG4gICAgICAgICAgICBxdWFkSWR4ID0gcXVhZElkeCArIDE7XG4gICAgICAgICAgICB2YXIgb2xkTGVmdCA9IGxlZnQ7XG4gICAgICAgICAgICBsZWZ0ID0gcmlnaHQ7XG4gICAgICAgICAgICByaWdodCA9IHJpZ2h0ICsgKHJpZ2h0IC0gb2xkTGVmdCk7XG4gICAgICAgICAgfVxuICAgICAgICAgIGlmICh5ID4gYm90dG9tKSB7IC8vIGFuZCBpbiBzb3V0aC5cbiAgICAgICAgICAgIHF1YWRJZHggPSBxdWFkSWR4ICsgMjtcbiAgICAgICAgICAgIHZhciBvbGRUb3AgPSB0b3A7XG4gICAgICAgICAgICB0b3AgPSBib3R0b207XG4gICAgICAgICAgICBib3R0b20gPSBib3R0b20gKyAoYm90dG9tIC0gb2xkVG9wKTtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICB2YXIgY2hpbGQgPSBnZXRDaGlsZChub2RlLCBxdWFkSWR4KTtcbiAgICAgICAgICBpZiAoIWNoaWxkKSB7XG4gICAgICAgICAgICAvLyBUaGUgbm9kZSBpcyBpbnRlcm5hbCBidXQgdGhpcyBxdWFkcmFudCBpcyBub3QgdGFrZW4uIEFkZFxuICAgICAgICAgICAgLy8gc3Vibm9kZSB0byBpdC5cbiAgICAgICAgICAgIGNoaWxkID0gbmV3Tm9kZSgpO1xuICAgICAgICAgICAgY2hpbGQubGVmdCA9IGxlZnQ7XG4gICAgICAgICAgICBjaGlsZC50b3AgPSB0b3A7XG4gICAgICAgICAgICBjaGlsZC5yaWdodCA9IHJpZ2h0O1xuICAgICAgICAgICAgY2hpbGQuYm90dG9tID0gYm90dG9tO1xuICAgICAgICAgICAgY2hpbGQuYm9keSA9IGJvZHk7XG5cbiAgICAgICAgICAgIHNldENoaWxkKG5vZGUsIHF1YWRJZHgsIGNoaWxkKTtcbiAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgLy8gY29udGludWUgc2VhcmNoaW5nIGluIHRoaXMgcXVhZHJhbnQuXG4gICAgICAgICAgICBpbnNlcnRTdGFjay5wdXNoKGNoaWxkLCBib2R5KTtcbiAgICAgICAgICB9XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgLy8gV2UgYXJlIHRyeWluZyB0byBhZGQgdG8gdGhlIGxlYWYgbm9kZS5cbiAgICAgICAgICAvLyBXZSBoYXZlIHRvIGNvbnZlcnQgY3VycmVudCBsZWFmIGludG8gaW50ZXJuYWwgbm9kZVxuICAgICAgICAgIC8vIGFuZCBjb250aW51ZSBhZGRpbmcgdHdvIG5vZGVzLlxuICAgICAgICAgIHZhciBvbGRCb2R5ID0gbm9kZS5ib2R5O1xuICAgICAgICAgIG5vZGUuYm9keSA9IG51bGw7IC8vIGludGVybmFsIG5vZGVzIGRvIG5vdCBjYXJ5IGJvZGllc1xuXG4gICAgICAgICAgaWYgKGlzU2FtZVBvc2l0aW9uKG9sZEJvZHkucG9zLCBib2R5LnBvcykpIHtcbiAgICAgICAgICAgIC8vIFByZXZlbnQgaW5maW5pdGUgc3ViZGl2aXNpb24gYnkgYnVtcGluZyBvbmUgbm9kZVxuICAgICAgICAgICAgLy8gYW55d2hlcmUgaW4gdGhpcyBxdWFkcmFudFxuICAgICAgICAgICAgdmFyIHJldHJpZXNDb3VudCA9IDM7XG4gICAgICAgICAgICBkbyB7XG4gICAgICAgICAgICAgIHZhciBvZmZzZXQgPSByYW5kb20ubmV4dERvdWJsZSgpO1xuICAgICAgICAgICAgICB2YXIgZHggPSAobm9kZS5yaWdodCAtIG5vZGUubGVmdCkgKiBvZmZzZXQ7XG4gICAgICAgICAgICAgIHZhciBkeSA9IChub2RlLmJvdHRvbSAtIG5vZGUudG9wKSAqIG9mZnNldDtcblxuICAgICAgICAgICAgICBvbGRCb2R5LnBvcy54ID0gbm9kZS5sZWZ0ICsgZHg7XG4gICAgICAgICAgICAgIG9sZEJvZHkucG9zLnkgPSBub2RlLnRvcCArIGR5O1xuICAgICAgICAgICAgICByZXRyaWVzQ291bnQgLT0gMTtcbiAgICAgICAgICAgICAgLy8gTWFrZSBzdXJlIHdlIGRvbid0IGJ1bXAgaXQgb3V0IG9mIHRoZSBib3guIElmIHdlIGRvLCBuZXh0IGl0ZXJhdGlvbiBzaG91bGQgZml4IGl0XG4gICAgICAgICAgICB9IHdoaWxlIChyZXRyaWVzQ291bnQgPiAwICYmIGlzU2FtZVBvc2l0aW9uKG9sZEJvZHkucG9zLCBib2R5LnBvcykpO1xuXG4gICAgICAgICAgICBpZiAocmV0cmllc0NvdW50ID09PSAwICYmIGlzU2FtZVBvc2l0aW9uKG9sZEJvZHkucG9zLCBib2R5LnBvcykpIHtcbiAgICAgICAgICAgICAgLy8gVGhpcyBpcyB2ZXJ5IGJhZCwgd2UgcmFuIG91dCBvZiBwcmVjaXNpb24uXG4gICAgICAgICAgICAgIC8vIGlmIHdlIGRvIG5vdCByZXR1cm4gZnJvbSB0aGUgbWV0aG9kIHdlJ2xsIGdldCBpbnRvXG4gICAgICAgICAgICAgIC8vIGluZmluaXRlIGxvb3AgaGVyZS4gU28gd2Ugc2FjcmlmaWNlIGNvcnJlY3RuZXNzIG9mIGxheW91dCwgYW5kIGtlZXAgdGhlIGFwcCBydW5uaW5nXG4gICAgICAgICAgICAgIC8vIE5leHQgbGF5b3V0IGl0ZXJhdGlvbiBzaG91bGQgZ2V0IGxhcmdlciBib3VuZGluZyBib3ggaW4gdGhlIGZpcnN0IHN0ZXAgYW5kIGZpeCB0aGlzXG4gICAgICAgICAgICAgIHJldHVybjtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgICAgLy8gTmV4dCBpdGVyYXRpb24gc2hvdWxkIHN1YmRpdmlkZSBub2RlIGZ1cnRoZXIuXG4gICAgICAgICAgaW5zZXJ0U3RhY2sucHVzaChub2RlLCBvbGRCb2R5KTtcbiAgICAgICAgICBpbnNlcnRTdGFjay5wdXNoKG5vZGUsIGJvZHkpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfSxcblxuICAgIHVwZGF0ZSA9IGZ1bmN0aW9uKHNvdXJjZUJvZHkpIHtcbiAgICAgIHZhciBxdWV1ZSA9IHVwZGF0ZVF1ZXVlLFxuICAgICAgICB2LFxuICAgICAgICBkeCxcbiAgICAgICAgZHksXG4gICAgICAgIHIsIGZ4ID0gMCxcbiAgICAgICAgZnkgPSAwLFxuICAgICAgICBxdWV1ZUxlbmd0aCA9IDEsXG4gICAgICAgIHNoaWZ0SWR4ID0gMCxcbiAgICAgICAgcHVzaElkeCA9IDE7XG5cbiAgICAgIHF1ZXVlWzBdID0gcm9vdDtcblxuICAgICAgd2hpbGUgKHF1ZXVlTGVuZ3RoKSB7XG4gICAgICAgIHZhciBub2RlID0gcXVldWVbc2hpZnRJZHhdLFxuICAgICAgICAgIGJvZHkgPSBub2RlLmJvZHk7XG5cbiAgICAgICAgcXVldWVMZW5ndGggLT0gMTtcbiAgICAgICAgc2hpZnRJZHggKz0gMTtcbiAgICAgICAgdmFyIGRpZmZlcmVudEJvZHkgPSAoYm9keSAhPT0gc291cmNlQm9keSk7XG4gICAgICAgIGlmIChib2R5ICYmIGRpZmZlcmVudEJvZHkpIHtcbiAgICAgICAgICAvLyBJZiB0aGUgY3VycmVudCBub2RlIGlzIGEgbGVhZiBub2RlIChhbmQgaXQgaXMgbm90IHNvdXJjZSBib2R5KSxcbiAgICAgICAgICAvLyBjYWxjdWxhdGUgdGhlIGZvcmNlIGV4ZXJ0ZWQgYnkgdGhlIGN1cnJlbnQgbm9kZSBvbiBib2R5LCBhbmQgYWRkIHRoaXNcbiAgICAgICAgICAvLyBhbW91bnQgdG8gYm9keSdzIG5ldCBmb3JjZS5cbiAgICAgICAgICBkeCA9IGJvZHkucG9zLnggLSBzb3VyY2VCb2R5LnBvcy54O1xuICAgICAgICAgIGR5ID0gYm9keS5wb3MueSAtIHNvdXJjZUJvZHkucG9zLnk7XG4gICAgICAgICAgciA9IE1hdGguc3FydChkeCAqIGR4ICsgZHkgKiBkeSk7XG5cbiAgICAgICAgICBpZiAociA9PT0gMCkge1xuICAgICAgICAgICAgLy8gUG9vciBtYW4ncyBwcm90ZWN0aW9uIGFnYWluc3QgemVybyBkaXN0YW5jZS5cbiAgICAgICAgICAgIGR4ID0gKHJhbmRvbS5uZXh0RG91YmxlKCkgLSAwLjUpIC8gNTA7XG4gICAgICAgICAgICBkeSA9IChyYW5kb20ubmV4dERvdWJsZSgpIC0gMC41KSAvIDUwO1xuICAgICAgICAgICAgciA9IE1hdGguc3FydChkeCAqIGR4ICsgZHkgKiBkeSk7XG4gICAgICAgICAgfVxuXG4gICAgICAgICAgLy8gVGhpcyBpcyBzdGFuZGFyZCBncmF2aXRpb24gZm9yY2UgY2FsY3VsYXRpb24gYnV0IHdlIGRpdmlkZVxuICAgICAgICAgIC8vIGJ5IHJeMyB0byBzYXZlIHR3byBvcGVyYXRpb25zIHdoZW4gbm9ybWFsaXppbmcgZm9yY2UgdmVjdG9yLlxuICAgICAgICAgIHYgPSBncmF2aXR5ICogYm9keS5tYXNzICogc291cmNlQm9keS5tYXNzIC8gKHIgKiByICogcik7XG4gICAgICAgICAgZnggKz0gdiAqIGR4O1xuICAgICAgICAgIGZ5ICs9IHYgKiBkeTtcbiAgICAgICAgfSBlbHNlIGlmIChkaWZmZXJlbnRCb2R5KSB7XG4gICAgICAgICAgLy8gT3RoZXJ3aXNlLCBjYWxjdWxhdGUgdGhlIHJhdGlvIHMgLyByLCAgd2hlcmUgcyBpcyB0aGUgd2lkdGggb2YgdGhlIHJlZ2lvblxuICAgICAgICAgIC8vIHJlcHJlc2VudGVkIGJ5IHRoZSBpbnRlcm5hbCBub2RlLCBhbmQgciBpcyB0aGUgZGlzdGFuY2UgYmV0d2VlbiB0aGUgYm9keVxuICAgICAgICAgIC8vIGFuZCB0aGUgbm9kZSdzIGNlbnRlci1vZi1tYXNzXG4gICAgICAgICAgZHggPSBub2RlLm1hc3NYIC8gbm9kZS5tYXNzIC0gc291cmNlQm9keS5wb3MueDtcbiAgICAgICAgICBkeSA9IG5vZGUubWFzc1kgLyBub2RlLm1hc3MgLSBzb3VyY2VCb2R5LnBvcy55O1xuICAgICAgICAgIHIgPSBNYXRoLnNxcnQoZHggKiBkeCArIGR5ICogZHkpO1xuXG4gICAgICAgICAgaWYgKHIgPT09IDApIHtcbiAgICAgICAgICAgIC8vIFNvcnJ5IGFib3V0IGNvZGUgZHVwbHVjYXRpb24uIEkgZG9uJ3Qgd2FudCB0byBjcmVhdGUgbWFueSBmdW5jdGlvbnNcbiAgICAgICAgICAgIC8vIHJpZ2h0IGF3YXkuIEp1c3Qgd2FudCB0byBzZWUgcGVyZm9ybWFuY2UgZmlyc3QuXG4gICAgICAgICAgICBkeCA9IChyYW5kb20ubmV4dERvdWJsZSgpIC0gMC41KSAvIDUwO1xuICAgICAgICAgICAgZHkgPSAocmFuZG9tLm5leHREb3VibGUoKSAtIDAuNSkgLyA1MDtcbiAgICAgICAgICAgIHIgPSBNYXRoLnNxcnQoZHggKiBkeCArIGR5ICogZHkpO1xuICAgICAgICAgIH1cbiAgICAgICAgICAvLyBJZiBzIC8gciA8IM64LCB0cmVhdCB0aGlzIGludGVybmFsIG5vZGUgYXMgYSBzaW5nbGUgYm9keSwgYW5kIGNhbGN1bGF0ZSB0aGVcbiAgICAgICAgICAvLyBmb3JjZSBpdCBleGVydHMgb24gc291cmNlQm9keSwgYW5kIGFkZCB0aGlzIGFtb3VudCB0byBzb3VyY2VCb2R5J3MgbmV0IGZvcmNlLlxuICAgICAgICAgIGlmICgobm9kZS5yaWdodCAtIG5vZGUubGVmdCkgLyByIDwgdGhldGEpIHtcbiAgICAgICAgICAgIC8vIGluIHRoZSBpZiBzdGF0ZW1lbnQgYWJvdmUgd2UgY29uc2lkZXIgbm9kZSdzIHdpZHRoIG9ubHlcbiAgICAgICAgICAgIC8vIGJlY2F1c2UgdGhlIHJlZ2lvbiB3YXMgc3F1YXJpZmllZCBkdXJpbmcgdHJlZSBjcmVhdGlvbi5cbiAgICAgICAgICAgIC8vIFRodXMgdGhlcmUgaXMgbm8gZGlmZmVyZW5jZSBiZXR3ZWVuIHVzaW5nIHdpZHRoIG9yIGhlaWdodC5cbiAgICAgICAgICAgIHYgPSBncmF2aXR5ICogbm9kZS5tYXNzICogc291cmNlQm9keS5tYXNzIC8gKHIgKiByICogcik7XG4gICAgICAgICAgICBmeCArPSB2ICogZHg7XG4gICAgICAgICAgICBmeSArPSB2ICogZHk7XG4gICAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIC8vIE90aGVyd2lzZSwgcnVuIHRoZSBwcm9jZWR1cmUgcmVjdXJzaXZlbHkgb24gZWFjaCBvZiB0aGUgY3VycmVudCBub2RlJ3MgY2hpbGRyZW4uXG5cbiAgICAgICAgICAgIC8vIEkgaW50ZW50aW9uYWxseSB1bmZvbGRlZCB0aGlzIGxvb3AsIHRvIHNhdmUgc2V2ZXJhbCBDUFUgY3ljbGVzLlxuICAgICAgICAgICAgaWYgKG5vZGUucXVhZDApIHtcbiAgICAgICAgICAgICAgcXVldWVbcHVzaElkeF0gPSBub2RlLnF1YWQwO1xuICAgICAgICAgICAgICBxdWV1ZUxlbmd0aCArPSAxO1xuICAgICAgICAgICAgICBwdXNoSWR4ICs9IDE7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpZiAobm9kZS5xdWFkMSkge1xuICAgICAgICAgICAgICBxdWV1ZVtwdXNoSWR4XSA9IG5vZGUucXVhZDE7XG4gICAgICAgICAgICAgIHF1ZXVlTGVuZ3RoICs9IDE7XG4gICAgICAgICAgICAgIHB1c2hJZHggKz0gMTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGlmIChub2RlLnF1YWQyKSB7XG4gICAgICAgICAgICAgIHF1ZXVlW3B1c2hJZHhdID0gbm9kZS5xdWFkMjtcbiAgICAgICAgICAgICAgcXVldWVMZW5ndGggKz0gMTtcbiAgICAgICAgICAgICAgcHVzaElkeCArPSAxO1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgaWYgKG5vZGUucXVhZDMpIHtcbiAgICAgICAgICAgICAgcXVldWVbcHVzaElkeF0gPSBub2RlLnF1YWQzO1xuICAgICAgICAgICAgICBxdWV1ZUxlbmd0aCArPSAxO1xuICAgICAgICAgICAgICBwdXNoSWR4ICs9IDE7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIHNvdXJjZUJvZHkuZm9yY2UueCArPSBmeDtcbiAgICAgIHNvdXJjZUJvZHkuZm9yY2UueSArPSBmeTtcbiAgICB9LFxuXG4gICAgaW5zZXJ0Qm9kaWVzID0gZnVuY3Rpb24oYm9kaWVzKSB7XG4gICAgICB2YXIgeDEgPSBOdW1iZXIuTUFYX1ZBTFVFLFxuICAgICAgICB5MSA9IE51bWJlci5NQVhfVkFMVUUsXG4gICAgICAgIHgyID0gTnVtYmVyLk1JTl9WQUxVRSxcbiAgICAgICAgeTIgPSBOdW1iZXIuTUlOX1ZBTFVFLFxuICAgICAgICBpLFxuICAgICAgICBtYXggPSBib2RpZXMubGVuZ3RoO1xuXG4gICAgICAvLyBUbyByZWR1Y2UgcXVhZCB0cmVlIGRlcHRoIHdlIGFyZSBsb29raW5nIGZvciBleGFjdCBib3VuZGluZyBib3ggb2YgYWxsIHBhcnRpY2xlcy5cbiAgICAgIGkgPSBtYXg7XG4gICAgICB3aGlsZSAoaS0tKSB7XG4gICAgICAgIHZhciB4ID0gYm9kaWVzW2ldLnBvcy54O1xuICAgICAgICB2YXIgeSA9IGJvZGllc1tpXS5wb3MueTtcbiAgICAgICAgaWYgKHggPCB4MSkge1xuICAgICAgICAgIHgxID0geDtcbiAgICAgICAgfVxuICAgICAgICBpZiAoeCA+IHgyKSB7XG4gICAgICAgICAgeDIgPSB4O1xuICAgICAgICB9XG4gICAgICAgIGlmICh5IDwgeTEpIHtcbiAgICAgICAgICB5MSA9IHk7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKHkgPiB5Mikge1xuICAgICAgICAgIHkyID0geTtcbiAgICAgICAgfVxuICAgICAgfVxuXG4gICAgICAvLyBTcXVhcmlmeSB0aGUgYm91bmRzLlxuICAgICAgdmFyIGR4ID0geDIgLSB4MSxcbiAgICAgICAgZHkgPSB5MiAtIHkxO1xuICAgICAgaWYgKGR4ID4gZHkpIHtcbiAgICAgICAgeTIgPSB5MSArIGR4O1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgeDIgPSB4MSArIGR5O1xuICAgICAgfVxuXG4gICAgICBjdXJyZW50SW5DYWNoZSA9IDA7XG4gICAgICByb290ID0gbmV3Tm9kZSgpO1xuICAgICAgcm9vdC5sZWZ0ID0geDE7XG4gICAgICByb290LnJpZ2h0ID0geDI7XG4gICAgICByb290LnRvcCA9IHkxO1xuICAgICAgcm9vdC5ib3R0b20gPSB5MjtcblxuICAgICAgaSA9IG1heCAtIDE7XG4gICAgICBpZiAoaSA+IDApIHtcbiAgICAgICAgcm9vdC5ib2R5ID0gYm9kaWVzW2ldO1xuICAgICAgfVxuICAgICAgd2hpbGUgKGktLSkge1xuICAgICAgICBpbnNlcnQoYm9kaWVzW2ldLCByb290KTtcbiAgICAgIH1cbiAgICB9O1xuXG4gIHJldHVybiB7XG4gICAgaW5zZXJ0Qm9kaWVzOiBpbnNlcnRCb2RpZXMsXG4gICAgdXBkYXRlQm9keUZvcmNlOiB1cGRhdGUsXG4gICAgb3B0aW9uczogZnVuY3Rpb24obmV3T3B0aW9ucykge1xuICAgICAgaWYgKG5ld09wdGlvbnMpIHtcbiAgICAgICAgaWYgKHR5cGVvZiBuZXdPcHRpb25zLmdyYXZpdHkgPT09ICdudW1iZXInKSB7XG4gICAgICAgICAgZ3Jhdml0eSA9IG5ld09wdGlvbnMuZ3Jhdml0eTtcbiAgICAgICAgfVxuICAgICAgICBpZiAodHlwZW9mIG5ld09wdGlvbnMudGhldGEgPT09ICdudW1iZXInKSB7XG4gICAgICAgICAgdGhldGEgPSBuZXdPcHRpb25zLnRoZXRhO1xuICAgICAgICB9XG5cbiAgICAgICAgcmV0dXJuIHRoaXM7XG4gICAgICB9XG5cbiAgICAgIHJldHVybiB7XG4gICAgICAgIGdyYXZpdHk6IGdyYXZpdHksXG4gICAgICAgIHRoZXRhOiB0aGV0YVxuICAgICAgfTtcbiAgICB9XG4gIH07XG59O1xuXG5mdW5jdGlvbiBnZXRDaGlsZChub2RlLCBpZHgpIHtcbiAgaWYgKGlkeCA9PT0gMCkgcmV0dXJuIG5vZGUucXVhZDA7XG4gIGlmIChpZHggPT09IDEpIHJldHVybiBub2RlLnF1YWQxO1xuICBpZiAoaWR4ID09PSAyKSByZXR1cm4gbm9kZS5xdWFkMjtcbiAgaWYgKGlkeCA9PT0gMykgcmV0dXJuIG5vZGUucXVhZDM7XG4gIHJldHVybiBudWxsO1xufVxuXG5mdW5jdGlvbiBzZXRDaGlsZChub2RlLCBpZHgsIGNoaWxkKSB7XG4gIGlmIChpZHggPT09IDApIG5vZGUucXVhZDAgPSBjaGlsZDtcbiAgZWxzZSBpZiAoaWR4ID09PSAxKSBub2RlLnF1YWQxID0gY2hpbGQ7XG4gIGVsc2UgaWYgKGlkeCA9PT0gMikgbm9kZS5xdWFkMiA9IGNoaWxkO1xuICBlbHNlIGlmIChpZHggPT09IDMpIG5vZGUucXVhZDMgPSBjaGlsZDtcbn1cbiIsIm1vZHVsZS5leHBvcnRzID0gSW5zZXJ0U3RhY2s7XG5cbi8qKlxuICogT3VyIGltcGxtZW50YXRpb24gb2YgUXVhZFRyZWUgaXMgbm9uLXJlY3Vyc2l2ZSB0byBhdm9pZCBHQyBoaXRcbiAqIFRoaXMgZGF0YSBzdHJ1Y3R1cmUgcmVwcmVzZW50IHN0YWNrIG9mIGVsZW1lbnRzXG4gKiB3aGljaCB3ZSBhcmUgdHJ5aW5nIHRvIGluc2VydCBpbnRvIHF1YWQgdHJlZS5cbiAqL1xuZnVuY3Rpb24gSW5zZXJ0U3RhY2sgKCkge1xuICAgIHRoaXMuc3RhY2sgPSBbXTtcbiAgICB0aGlzLnBvcElkeCA9IDA7XG59XG5cbkluc2VydFN0YWNrLnByb3RvdHlwZSA9IHtcbiAgICBpc0VtcHR5OiBmdW5jdGlvbigpIHtcbiAgICAgICAgcmV0dXJuIHRoaXMucG9wSWR4ID09PSAwO1xuICAgIH0sXG4gICAgcHVzaDogZnVuY3Rpb24gKG5vZGUsIGJvZHkpIHtcbiAgICAgICAgdmFyIGl0ZW0gPSB0aGlzLnN0YWNrW3RoaXMucG9wSWR4XTtcbiAgICAgICAgaWYgKCFpdGVtKSB7XG4gICAgICAgICAgICAvLyB3ZSBhcmUgdHJ5aW5nIHRvIGF2b2lkIG1lbW9yeSBwcmVzc3VlOiBjcmVhdGUgbmV3IGVsZW1lbnRcbiAgICAgICAgICAgIC8vIG9ubHkgd2hlbiBhYnNvbHV0ZWx5IG5lY2Vzc2FyeVxuICAgICAgICAgICAgdGhpcy5zdGFja1t0aGlzLnBvcElkeF0gPSBuZXcgSW5zZXJ0U3RhY2tFbGVtZW50KG5vZGUsIGJvZHkpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgaXRlbS5ub2RlID0gbm9kZTtcbiAgICAgICAgICAgIGl0ZW0uYm9keSA9IGJvZHk7XG4gICAgICAgIH1cbiAgICAgICAgKyt0aGlzLnBvcElkeDtcbiAgICB9LFxuICAgIHBvcDogZnVuY3Rpb24gKCkge1xuICAgICAgICBpZiAodGhpcy5wb3BJZHggPiAwKSB7XG4gICAgICAgICAgICByZXR1cm4gdGhpcy5zdGFja1stLXRoaXMucG9wSWR4XTtcbiAgICAgICAgfVxuICAgIH0sXG4gICAgcmVzZXQ6IGZ1bmN0aW9uICgpIHtcbiAgICAgICAgdGhpcy5wb3BJZHggPSAwO1xuICAgIH1cbn07XG5cbmZ1bmN0aW9uIEluc2VydFN0YWNrRWxlbWVudChub2RlLCBib2R5KSB7XG4gICAgdGhpcy5ub2RlID0gbm9kZTsgLy8gUXVhZFRyZWUgbm9kZVxuICAgIHRoaXMuYm9keSA9IGJvZHk7IC8vIHBoeXNpY2FsIGJvZHkgd2hpY2ggbmVlZHMgdG8gYmUgaW5zZXJ0ZWQgdG8gbm9kZVxufVxuIiwibW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiBpc1NhbWVQb3NpdGlvbihwb2ludDEsIHBvaW50Mikge1xuICAgIHZhciBkeCA9IE1hdGguYWJzKHBvaW50MS54IC0gcG9pbnQyLngpO1xuICAgIHZhciBkeSA9IE1hdGguYWJzKHBvaW50MS55IC0gcG9pbnQyLnkpO1xuXG4gICAgcmV0dXJuIChkeCA8IDFlLTggJiYgZHkgPCAxZS04KTtcbn07XG4iLCIvKipcbiAqIEludGVybmFsIGRhdGEgc3RydWN0dXJlIHRvIHJlcHJlc2VudCAyRCBRdWFkVHJlZSBub2RlXG4gKi9cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gTm9kZSgpIHtcbiAgLy8gYm9keSBzdG9yZWQgaW5zaWRlIHRoaXMgbm9kZS4gSW4gcXVhZCB0cmVlIG9ubHkgbGVhZiBub2RlcyAoYnkgY29uc3RydWN0aW9uKVxuICAvLyBjb250YWluIGJvaWRlczpcbiAgdGhpcy5ib2R5ID0gbnVsbDtcblxuICAvLyBDaGlsZCBub2RlcyBhcmUgc3RvcmVkIGluIHF1YWRzLiBFYWNoIHF1YWQgaXMgcHJlc2VudGVkIGJ5IG51bWJlcjpcbiAgLy8gMCB8IDFcbiAgLy8gLS0tLS1cbiAgLy8gMiB8IDNcbiAgdGhpcy5xdWFkMCA9IG51bGw7XG4gIHRoaXMucXVhZDEgPSBudWxsO1xuICB0aGlzLnF1YWQyID0gbnVsbDtcbiAgdGhpcy5xdWFkMyA9IG51bGw7XG5cbiAgLy8gVG90YWwgbWFzcyBvZiBjdXJyZW50IG5vZGVcbiAgdGhpcy5tYXNzID0gMDtcblxuICAvLyBDZW50ZXIgb2YgbWFzcyBjb29yZGluYXRlc1xuICB0aGlzLm1hc3NYID0gMDtcbiAgdGhpcy5tYXNzWSA9IDA7XG5cbiAgLy8gYm91bmRpbmcgYm94IGNvb3JkaW5hdGVzXG4gIHRoaXMubGVmdCA9IDA7XG4gIHRoaXMudG9wID0gMDtcbiAgdGhpcy5ib3R0b20gPSAwO1xuICB0aGlzLnJpZ2h0ID0gMDtcbn07XG4iLCIvKipcbiAqIFRoaXMgaXMgQmFybmVzIEh1dCBzaW11bGF0aW9uIGFsZ29yaXRobSBmb3IgMmQgY2FzZS4gSW1wbGVtZW50YXRpb25cbiAqIGlzIGhpZ2hseSBvcHRpbWl6ZWQgKGF2b2lkcyByZWN1c2lvbiBhbmQgZ2MgcHJlc3N1cmUpXG4gKlxuICogaHR0cDovL3d3dy5jcy5wcmluY2V0b24uZWR1L2NvdXJzZXMvYXJjaGl2ZS9mYWxsMDMvY3MxMjYvYXNzaWdubWVudHMvYmFybmVzLWh1dC5odG1sXG4gKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbihvcHRpb25zKSB7XG4gIG9wdGlvbnMgPSBvcHRpb25zIHx8IHt9O1xuICBvcHRpb25zLmdyYXZpdHkgPSB0eXBlb2Ygb3B0aW9ucy5ncmF2aXR5ID09PSAnbnVtYmVyJyA/IG9wdGlvbnMuZ3Jhdml0eSA6IC0xO1xuICBvcHRpb25zLnRoZXRhID0gdHlwZW9mIG9wdGlvbnMudGhldGEgPT09ICdudW1iZXInID8gb3B0aW9ucy50aGV0YSA6IDAuODtcblxuICAvLyB3ZSByZXF1aXJlIGRldGVybWluaXN0aWMgcmFuZG9tbmVzcyBoZXJlXG4gIHZhciByYW5kb20gPSByZXF1aXJlKCduZ3JhcGgucmFuZG9tJykucmFuZG9tKDE5ODQpLFxuICAgIE5vZGUgPSByZXF1aXJlKCcuL25vZGUnKSxcbiAgICBJbnNlcnRTdGFjayA9IHJlcXVpcmUoJy4vaW5zZXJ0U3RhY2snKSxcbiAgICBpc1NhbWVQb3NpdGlvbiA9IHJlcXVpcmUoJy4vaXNTYW1lUG9zaXRpb24nKTtcblxuICB2YXIgZ3Jhdml0eSA9IG9wdGlvbnMuZ3Jhdml0eSxcbiAgICB1cGRhdGVRdWV1ZSA9IFtdLFxuICAgIGluc2VydFN0YWNrID0gbmV3IEluc2VydFN0YWNrKCksXG4gICAgdGhldGEgPSBvcHRpb25zLnRoZXRhLFxuXG4gICAgbm9kZXNDYWNoZSA9IFtdLFxuICAgIGN1cnJlbnRJbkNhY2hlID0gMCxcbiAgICByb290ID0gbmV3Tm9kZSgpO1xuXG4gIHJldHVybiB7XG4gICAgaW5zZXJ0Qm9kaWVzOiBpbnNlcnRCb2RpZXMsXG4gICAgLyoqXG4gICAgICogR2V0cyByb290IG5vZGUgaWYgaXRzIHByZXNlbnRcbiAgICAgKi9cbiAgICBnZXRSb290OiBmdW5jdGlvbigpIHtcbiAgICAgIHJldHVybiByb290O1xuICAgIH0sXG4gICAgdXBkYXRlQm9keUZvcmNlOiB1cGRhdGUsXG4gICAgb3B0aW9uczogZnVuY3Rpb24obmV3T3B0aW9ucykge1xuICAgICAgaWYgKG5ld09wdGlvbnMpIHtcbiAgICAgICAgaWYgKHR5cGVvZiBuZXdPcHRpb25zLmdyYXZpdHkgPT09ICdudW1iZXInKSB7XG4gICAgICAgICAgZ3Jhdml0eSA9IG5ld09wdGlvbnMuZ3Jhdml0eTtcbiAgICAgICAgfVxuICAgICAgICBpZiAodHlwZW9mIG5ld09wdGlvbnMudGhldGEgPT09ICdudW1iZXInKSB7XG4gICAgICAgICAgdGhldGEgPSBuZXdPcHRpb25zLnRoZXRhO1xuICAgICAgICB9XG5cbiAgICAgICAgcmV0dXJuIHRoaXM7XG4gICAgICB9XG5cbiAgICAgIHJldHVybiB7XG4gICAgICAgIGdyYXZpdHk6IGdyYXZpdHksXG4gICAgICAgIHRoZXRhOiB0aGV0YVxuICAgICAgfTtcbiAgICB9XG4gIH07XG5cbiAgZnVuY3Rpb24gbmV3Tm9kZSgpIHtcbiAgICAvLyBUbyBhdm9pZCBwcmVzc3VyZSBvbiBHQyB3ZSByZXVzZSBub2Rlcy5cbiAgICB2YXIgbm9kZSA9IG5vZGVzQ2FjaGVbY3VycmVudEluQ2FjaGVdO1xuICAgIGlmIChub2RlKSB7XG4gICAgICBub2RlLnF1YWQwID0gbnVsbDtcbiAgICAgIG5vZGUucXVhZDEgPSBudWxsO1xuICAgICAgbm9kZS5xdWFkMiA9IG51bGw7XG4gICAgICBub2RlLnF1YWQzID0gbnVsbDtcbiAgICAgIG5vZGUuYm9keSA9IG51bGw7XG4gICAgICBub2RlLm1hc3MgPSBub2RlLm1hc3NYID0gbm9kZS5tYXNzWSA9IDA7XG4gICAgICBub2RlLmxlZnQgPSBub2RlLnJpZ2h0ID0gbm9kZS50b3AgPSBub2RlLmJvdHRvbSA9IDA7XG4gICAgfSBlbHNlIHtcbiAgICAgIG5vZGUgPSBuZXcgTm9kZSgpO1xuICAgICAgbm9kZXNDYWNoZVtjdXJyZW50SW5DYWNoZV0gPSBub2RlO1xuICAgIH1cblxuICAgICsrY3VycmVudEluQ2FjaGU7XG4gICAgcmV0dXJuIG5vZGU7XG4gIH1cblxuICBmdW5jdGlvbiB1cGRhdGUoc291cmNlQm9keSkge1xuICAgIHZhciBxdWV1ZSA9IHVwZGF0ZVF1ZXVlLFxuICAgICAgdixcbiAgICAgIGR4LFxuICAgICAgZHksXG4gICAgICByLCBmeCA9IDAsXG4gICAgICBmeSA9IDAsXG4gICAgICBxdWV1ZUxlbmd0aCA9IDEsXG4gICAgICBzaGlmdElkeCA9IDAsXG4gICAgICBwdXNoSWR4ID0gMTtcblxuICAgIHF1ZXVlWzBdID0gcm9vdDtcblxuICAgIHdoaWxlIChxdWV1ZUxlbmd0aCkge1xuICAgICAgdmFyIG5vZGUgPSBxdWV1ZVtzaGlmdElkeF0sXG4gICAgICAgIGJvZHkgPSBub2RlLmJvZHk7XG5cbiAgICAgIHF1ZXVlTGVuZ3RoIC09IDE7XG4gICAgICBzaGlmdElkeCArPSAxO1xuICAgICAgdmFyIGRpZmZlcmVudEJvZHkgPSAoYm9keSAhPT0gc291cmNlQm9keSk7XG4gICAgICBpZiAoYm9keSAmJiBkaWZmZXJlbnRCb2R5KSB7XG4gICAgICAgIC8vIElmIHRoZSBjdXJyZW50IG5vZGUgaXMgYSBsZWFmIG5vZGUgKGFuZCBpdCBpcyBub3Qgc291cmNlIGJvZHkpLFxuICAgICAgICAvLyBjYWxjdWxhdGUgdGhlIGZvcmNlIGV4ZXJ0ZWQgYnkgdGhlIGN1cnJlbnQgbm9kZSBvbiBib2R5LCBhbmQgYWRkIHRoaXNcbiAgICAgICAgLy8gYW1vdW50IHRvIGJvZHkncyBuZXQgZm9yY2UuXG4gICAgICAgIGR4ID0gYm9keS5wb3MueCAtIHNvdXJjZUJvZHkucG9zLng7XG4gICAgICAgIGR5ID0gYm9keS5wb3MueSAtIHNvdXJjZUJvZHkucG9zLnk7XG4gICAgICAgIHIgPSBNYXRoLnNxcnQoZHggKiBkeCArIGR5ICogZHkpO1xuXG4gICAgICAgIGlmIChyID09PSAwKSB7XG4gICAgICAgICAgLy8gUG9vciBtYW4ncyBwcm90ZWN0aW9uIGFnYWluc3QgemVybyBkaXN0YW5jZS5cbiAgICAgICAgICBkeCA9IChyYW5kb20ubmV4dERvdWJsZSgpIC0gMC41KSAvIDUwO1xuICAgICAgICAgIGR5ID0gKHJhbmRvbS5uZXh0RG91YmxlKCkgLSAwLjUpIC8gNTA7XG4gICAgICAgICAgciA9IE1hdGguc3FydChkeCAqIGR4ICsgZHkgKiBkeSk7XG4gICAgICAgIH1cblxuICAgICAgICAvLyBUaGlzIGlzIHN0YW5kYXJkIGdyYXZpdGlvbiBmb3JjZSBjYWxjdWxhdGlvbiBidXQgd2UgZGl2aWRlXG4gICAgICAgIC8vIGJ5IHJeMyB0byBzYXZlIHR3byBvcGVyYXRpb25zIHdoZW4gbm9ybWFsaXppbmcgZm9yY2UgdmVjdG9yLlxuICAgICAgICB2ID0gZ3Jhdml0eSAqIGJvZHkubWFzcyAqIHNvdXJjZUJvZHkubWFzcyAvIChyICogciAqIHIpO1xuICAgICAgICBmeCArPSB2ICogZHg7XG4gICAgICAgIGZ5ICs9IHYgKiBkeTtcbiAgICAgIH0gZWxzZSBpZiAoZGlmZmVyZW50Qm9keSkge1xuICAgICAgICAvLyBPdGhlcndpc2UsIGNhbGN1bGF0ZSB0aGUgcmF0aW8gcyAvIHIsICB3aGVyZSBzIGlzIHRoZSB3aWR0aCBvZiB0aGUgcmVnaW9uXG4gICAgICAgIC8vIHJlcHJlc2VudGVkIGJ5IHRoZSBpbnRlcm5hbCBub2RlLCBhbmQgciBpcyB0aGUgZGlzdGFuY2UgYmV0d2VlbiB0aGUgYm9keVxuICAgICAgICAvLyBhbmQgdGhlIG5vZGUncyBjZW50ZXItb2YtbWFzc1xuICAgICAgICBkeCA9IG5vZGUubWFzc1ggLyBub2RlLm1hc3MgLSBzb3VyY2VCb2R5LnBvcy54O1xuICAgICAgICBkeSA9IG5vZGUubWFzc1kgLyBub2RlLm1hc3MgLSBzb3VyY2VCb2R5LnBvcy55O1xuICAgICAgICByID0gTWF0aC5zcXJ0KGR4ICogZHggKyBkeSAqIGR5KTtcblxuICAgICAgICBpZiAociA9PT0gMCkge1xuICAgICAgICAgIC8vIFNvcnJ5IGFib3V0IGNvZGUgZHVwbHVjYXRpb24uIEkgZG9uJ3Qgd2FudCB0byBjcmVhdGUgbWFueSBmdW5jdGlvbnNcbiAgICAgICAgICAvLyByaWdodCBhd2F5LiBKdXN0IHdhbnQgdG8gc2VlIHBlcmZvcm1hbmNlIGZpcnN0LlxuICAgICAgICAgIGR4ID0gKHJhbmRvbS5uZXh0RG91YmxlKCkgLSAwLjUpIC8gNTA7XG4gICAgICAgICAgZHkgPSAocmFuZG9tLm5leHREb3VibGUoKSAtIDAuNSkgLyA1MDtcbiAgICAgICAgICByID0gTWF0aC5zcXJ0KGR4ICogZHggKyBkeSAqIGR5KTtcbiAgICAgICAgfVxuICAgICAgICAvLyBJZiBzIC8gciA8IM64LCB0cmVhdCB0aGlzIGludGVybmFsIG5vZGUgYXMgYSBzaW5nbGUgYm9keSwgYW5kIGNhbGN1bGF0ZSB0aGVcbiAgICAgICAgLy8gZm9yY2UgaXQgZXhlcnRzIG9uIHNvdXJjZUJvZHksIGFuZCBhZGQgdGhpcyBhbW91bnQgdG8gc291cmNlQm9keSdzIG5ldCBmb3JjZS5cbiAgICAgICAgaWYgKChub2RlLnJpZ2h0IC0gbm9kZS5sZWZ0KSAvIHIgPCB0aGV0YSkge1xuICAgICAgICAgIC8vIGluIHRoZSBpZiBzdGF0ZW1lbnQgYWJvdmUgd2UgY29uc2lkZXIgbm9kZSdzIHdpZHRoIG9ubHlcbiAgICAgICAgICAvLyBiZWNhdXNlIHRoZSByZWdpb24gd2FzIHNxdWFyaWZpZWQgZHVyaW5nIHRyZWUgY3JlYXRpb24uXG4gICAgICAgICAgLy8gVGh1cyB0aGVyZSBpcyBubyBkaWZmZXJlbmNlIGJldHdlZW4gdXNpbmcgd2lkdGggb3IgaGVpZ2h0LlxuICAgICAgICAgIHYgPSBncmF2aXR5ICogbm9kZS5tYXNzICogc291cmNlQm9keS5tYXNzIC8gKHIgKiByICogcik7XG4gICAgICAgICAgZnggKz0gdiAqIGR4O1xuICAgICAgICAgIGZ5ICs9IHYgKiBkeTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAvLyBPdGhlcndpc2UsIHJ1biB0aGUgcHJvY2VkdXJlIHJlY3Vyc2l2ZWx5IG9uIGVhY2ggb2YgdGhlIGN1cnJlbnQgbm9kZSdzIGNoaWxkcmVuLlxuXG4gICAgICAgICAgLy8gSSBpbnRlbnRpb25hbGx5IHVuZm9sZGVkIHRoaXMgbG9vcCwgdG8gc2F2ZSBzZXZlcmFsIENQVSBjeWNsZXMuXG4gICAgICAgICAgaWYgKG5vZGUucXVhZDApIHtcbiAgICAgICAgICAgIHF1ZXVlW3B1c2hJZHhdID0gbm9kZS5xdWFkMDtcbiAgICAgICAgICAgIHF1ZXVlTGVuZ3RoICs9IDE7XG4gICAgICAgICAgICBwdXNoSWR4ICs9IDE7XG4gICAgICAgICAgfVxuICAgICAgICAgIGlmIChub2RlLnF1YWQxKSB7XG4gICAgICAgICAgICBxdWV1ZVtwdXNoSWR4XSA9IG5vZGUucXVhZDE7XG4gICAgICAgICAgICBxdWV1ZUxlbmd0aCArPSAxO1xuICAgICAgICAgICAgcHVzaElkeCArPSAxO1xuICAgICAgICAgIH1cbiAgICAgICAgICBpZiAobm9kZS5xdWFkMikge1xuICAgICAgICAgICAgcXVldWVbcHVzaElkeF0gPSBub2RlLnF1YWQyO1xuICAgICAgICAgICAgcXVldWVMZW5ndGggKz0gMTtcbiAgICAgICAgICAgIHB1c2hJZHggKz0gMTtcbiAgICAgICAgICB9XG4gICAgICAgICAgaWYgKG5vZGUucXVhZDMpIHtcbiAgICAgICAgICAgIHF1ZXVlW3B1c2hJZHhdID0gbm9kZS5xdWFkMztcbiAgICAgICAgICAgIHF1ZXVlTGVuZ3RoICs9IDE7XG4gICAgICAgICAgICBwdXNoSWR4ICs9IDE7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgc291cmNlQm9keS5mb3JjZS54ICs9IGZ4O1xuICAgIHNvdXJjZUJvZHkuZm9yY2UueSArPSBmeTtcbiAgfVxuXG4gIGZ1bmN0aW9uIGluc2VydEJvZGllcyhib2RpZXMpIHtcbiAgICB2YXIgeDEgPSBOdW1iZXIuTUFYX1ZBTFVFLFxuICAgICAgeTEgPSBOdW1iZXIuTUFYX1ZBTFVFLFxuICAgICAgeDIgPSBOdW1iZXIuTUlOX1ZBTFVFLFxuICAgICAgeTIgPSBOdW1iZXIuTUlOX1ZBTFVFLFxuICAgICAgaSxcbiAgICAgIG1heCA9IGJvZGllcy5sZW5ndGg7XG5cbiAgICAvLyBUbyByZWR1Y2UgcXVhZCB0cmVlIGRlcHRoIHdlIGFyZSBsb29raW5nIGZvciBleGFjdCBib3VuZGluZyBib3ggb2YgYWxsIHBhcnRpY2xlcy5cbiAgICBpID0gbWF4O1xuICAgIHdoaWxlIChpLS0pIHtcbiAgICAgIHZhciB4ID0gYm9kaWVzW2ldLnBvcy54O1xuICAgICAgdmFyIHkgPSBib2RpZXNbaV0ucG9zLnk7XG4gICAgICBpZiAoeCA8IHgxKSB7XG4gICAgICAgIHgxID0geDtcbiAgICAgIH1cbiAgICAgIGlmICh4ID4geDIpIHtcbiAgICAgICAgeDIgPSB4O1xuICAgICAgfVxuICAgICAgaWYgKHkgPCB5MSkge1xuICAgICAgICB5MSA9IHk7XG4gICAgICB9XG4gICAgICBpZiAoeSA+IHkyKSB7XG4gICAgICAgIHkyID0geTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICAvLyBTcXVhcmlmeSB0aGUgYm91bmRzLlxuICAgIHZhciBkeCA9IHgyIC0geDEsXG4gICAgICBkeSA9IHkyIC0geTE7XG4gICAgaWYgKGR4ID4gZHkpIHtcbiAgICAgIHkyID0geTEgKyBkeDtcbiAgICB9IGVsc2Uge1xuICAgICAgeDIgPSB4MSArIGR5O1xuICAgIH1cblxuICAgIGN1cnJlbnRJbkNhY2hlID0gMDtcbiAgICByb290ID0gbmV3Tm9kZSgpO1xuICAgIHJvb3QubGVmdCA9IHgxO1xuICAgIHJvb3QucmlnaHQgPSB4MjtcbiAgICByb290LnRvcCA9IHkxO1xuICAgIHJvb3QuYm90dG9tID0geTI7XG5cbiAgICBpID0gbWF4IC0gMTtcbiAgICBpZiAoaSA+PSAwKSB7XG4gICAgICByb290LmJvZHkgPSBib2RpZXNbaV07XG4gICAgfVxuICAgIHdoaWxlIChpLS0pIHtcbiAgICAgIGluc2VydChib2RpZXNbaV0sIHJvb3QpO1xuICAgIH1cbiAgfVxuXG4gIGZ1bmN0aW9uIGluc2VydChuZXdCb2R5KSB7XG4gICAgaW5zZXJ0U3RhY2sucmVzZXQoKTtcbiAgICBpbnNlcnRTdGFjay5wdXNoKHJvb3QsIG5ld0JvZHkpO1xuXG4gICAgd2hpbGUgKCFpbnNlcnRTdGFjay5pc0VtcHR5KCkpIHtcbiAgICAgIHZhciBzdGFja0l0ZW0gPSBpbnNlcnRTdGFjay5wb3AoKSxcbiAgICAgICAgbm9kZSA9IHN0YWNrSXRlbS5ub2RlLFxuICAgICAgICBib2R5ID0gc3RhY2tJdGVtLmJvZHk7XG5cbiAgICAgIGlmICghbm9kZS5ib2R5KSB7XG4gICAgICAgIC8vIFRoaXMgaXMgaW50ZXJuYWwgbm9kZS4gVXBkYXRlIHRoZSB0b3RhbCBtYXNzIG9mIHRoZSBub2RlIGFuZCBjZW50ZXItb2YtbWFzcy5cbiAgICAgICAgdmFyIHggPSBib2R5LnBvcy54O1xuICAgICAgICB2YXIgeSA9IGJvZHkucG9zLnk7XG4gICAgICAgIG5vZGUubWFzcyA9IG5vZGUubWFzcyArIGJvZHkubWFzcztcbiAgICAgICAgbm9kZS5tYXNzWCA9IG5vZGUubWFzc1ggKyBib2R5Lm1hc3MgKiB4O1xuICAgICAgICBub2RlLm1hc3NZID0gbm9kZS5tYXNzWSArIGJvZHkubWFzcyAqIHk7XG5cbiAgICAgICAgLy8gUmVjdXJzaXZlbHkgaW5zZXJ0IHRoZSBib2R5IGluIHRoZSBhcHByb3ByaWF0ZSBxdWFkcmFudC5cbiAgICAgICAgLy8gQnV0IGZpcnN0IGZpbmQgdGhlIGFwcHJvcHJpYXRlIHF1YWRyYW50LlxuICAgICAgICB2YXIgcXVhZElkeCA9IDAsIC8vIEFzc3VtZSB3ZSBhcmUgaW4gdGhlIDAncyBxdWFkLlxuICAgICAgICAgIGxlZnQgPSBub2RlLmxlZnQsXG4gICAgICAgICAgcmlnaHQgPSAobm9kZS5yaWdodCArIGxlZnQpIC8gMixcbiAgICAgICAgICB0b3AgPSBub2RlLnRvcCxcbiAgICAgICAgICBib3R0b20gPSAobm9kZS5ib3R0b20gKyB0b3ApIC8gMjtcblxuICAgICAgICBpZiAoeCA+IHJpZ2h0KSB7IC8vIHNvbWV3aGVyZSBpbiB0aGUgZWFzdGVybiBwYXJ0LlxuICAgICAgICAgIHF1YWRJZHggPSBxdWFkSWR4ICsgMTtcbiAgICAgICAgICBsZWZ0ID0gcmlnaHQ7XG4gICAgICAgICAgcmlnaHQgPSBub2RlLnJpZ2h0O1xuICAgICAgICB9XG4gICAgICAgIGlmICh5ID4gYm90dG9tKSB7IC8vIGFuZCBpbiBzb3V0aC5cbiAgICAgICAgICBxdWFkSWR4ID0gcXVhZElkeCArIDI7XG4gICAgICAgICAgdG9wID0gYm90dG9tO1xuICAgICAgICAgIGJvdHRvbSA9IG5vZGUuYm90dG9tO1xuICAgICAgICB9XG5cbiAgICAgICAgdmFyIGNoaWxkID0gZ2V0Q2hpbGQobm9kZSwgcXVhZElkeCk7XG4gICAgICAgIGlmICghY2hpbGQpIHtcbiAgICAgICAgICAvLyBUaGUgbm9kZSBpcyBpbnRlcm5hbCBidXQgdGhpcyBxdWFkcmFudCBpcyBub3QgdGFrZW4uIEFkZFxuICAgICAgICAgIC8vIHN1Ym5vZGUgdG8gaXQuXG4gICAgICAgICAgY2hpbGQgPSBuZXdOb2RlKCk7XG4gICAgICAgICAgY2hpbGQubGVmdCA9IGxlZnQ7XG4gICAgICAgICAgY2hpbGQudG9wID0gdG9wO1xuICAgICAgICAgIGNoaWxkLnJpZ2h0ID0gcmlnaHQ7XG4gICAgICAgICAgY2hpbGQuYm90dG9tID0gYm90dG9tO1xuICAgICAgICAgIGNoaWxkLmJvZHkgPSBib2R5O1xuXG4gICAgICAgICAgc2V0Q2hpbGQobm9kZSwgcXVhZElkeCwgY2hpbGQpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIC8vIGNvbnRpbnVlIHNlYXJjaGluZyBpbiB0aGlzIHF1YWRyYW50LlxuICAgICAgICAgIGluc2VydFN0YWNrLnB1c2goY2hpbGQsIGJvZHkpO1xuICAgICAgICB9XG4gICAgICB9IGVsc2Uge1xuICAgICAgICAvLyBXZSBhcmUgdHJ5aW5nIHRvIGFkZCB0byB0aGUgbGVhZiBub2RlLlxuICAgICAgICAvLyBXZSBoYXZlIHRvIGNvbnZlcnQgY3VycmVudCBsZWFmIGludG8gaW50ZXJuYWwgbm9kZVxuICAgICAgICAvLyBhbmQgY29udGludWUgYWRkaW5nIHR3byBub2Rlcy5cbiAgICAgICAgdmFyIG9sZEJvZHkgPSBub2RlLmJvZHk7XG4gICAgICAgIG5vZGUuYm9keSA9IG51bGw7IC8vIGludGVybmFsIG5vZGVzIGRvIG5vdCBjYXJ5IGJvZGllc1xuXG4gICAgICAgIGlmIChpc1NhbWVQb3NpdGlvbihvbGRCb2R5LnBvcywgYm9keS5wb3MpKSB7XG4gICAgICAgICAgLy8gUHJldmVudCBpbmZpbml0ZSBzdWJkaXZpc2lvbiBieSBidW1waW5nIG9uZSBub2RlXG4gICAgICAgICAgLy8gYW55d2hlcmUgaW4gdGhpcyBxdWFkcmFudFxuICAgICAgICAgIHZhciByZXRyaWVzQ291bnQgPSAzO1xuICAgICAgICAgIGRvIHtcbiAgICAgICAgICAgIHZhciBvZmZzZXQgPSByYW5kb20ubmV4dERvdWJsZSgpO1xuICAgICAgICAgICAgdmFyIGR4ID0gKG5vZGUucmlnaHQgLSBub2RlLmxlZnQpICogb2Zmc2V0O1xuICAgICAgICAgICAgdmFyIGR5ID0gKG5vZGUuYm90dG9tIC0gbm9kZS50b3ApICogb2Zmc2V0O1xuXG4gICAgICAgICAgICBvbGRCb2R5LnBvcy54ID0gbm9kZS5sZWZ0ICsgZHg7XG4gICAgICAgICAgICBvbGRCb2R5LnBvcy55ID0gbm9kZS50b3AgKyBkeTtcbiAgICAgICAgICAgIHJldHJpZXNDb3VudCAtPSAxO1xuICAgICAgICAgICAgLy8gTWFrZSBzdXJlIHdlIGRvbid0IGJ1bXAgaXQgb3V0IG9mIHRoZSBib3guIElmIHdlIGRvLCBuZXh0IGl0ZXJhdGlvbiBzaG91bGQgZml4IGl0XG4gICAgICAgICAgfSB3aGlsZSAocmV0cmllc0NvdW50ID4gMCAmJiBpc1NhbWVQb3NpdGlvbihvbGRCb2R5LnBvcywgYm9keS5wb3MpKTtcblxuICAgICAgICAgIGlmIChyZXRyaWVzQ291bnQgPT09IDAgJiYgaXNTYW1lUG9zaXRpb24ob2xkQm9keS5wb3MsIGJvZHkucG9zKSkge1xuICAgICAgICAgICAgLy8gVGhpcyBpcyB2ZXJ5IGJhZCwgd2UgcmFuIG91dCBvZiBwcmVjaXNpb24uXG4gICAgICAgICAgICAvLyBpZiB3ZSBkbyBub3QgcmV0dXJuIGZyb20gdGhlIG1ldGhvZCB3ZSdsbCBnZXQgaW50b1xuICAgICAgICAgICAgLy8gaW5maW5pdGUgbG9vcCBoZXJlLiBTbyB3ZSBzYWNyaWZpY2UgY29ycmVjdG5lc3Mgb2YgbGF5b3V0LCBhbmQga2VlcCB0aGUgYXBwIHJ1bm5pbmdcbiAgICAgICAgICAgIC8vIE5leHQgbGF5b3V0IGl0ZXJhdGlvbiBzaG91bGQgZ2V0IGxhcmdlciBib3VuZGluZyBib3ggaW4gdGhlIGZpcnN0IHN0ZXAgYW5kIGZpeCB0aGlzXG4gICAgICAgICAgICByZXR1cm47XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIC8vIE5leHQgaXRlcmF0aW9uIHNob3VsZCBzdWJkaXZpZGUgbm9kZSBmdXJ0aGVyLlxuICAgICAgICBpbnNlcnRTdGFjay5wdXNoKG5vZGUsIG9sZEJvZHkpO1xuICAgICAgICBpbnNlcnRTdGFjay5wdXNoKG5vZGUsIGJvZHkpO1xuICAgICAgfVxuICAgIH1cbiAgfVxufTtcblxuZnVuY3Rpb24gZ2V0Q2hpbGQobm9kZSwgaWR4KSB7XG4gIGlmIChpZHggPT09IDApIHJldHVybiBub2RlLnF1YWQwO1xuICBpZiAoaWR4ID09PSAxKSByZXR1cm4gbm9kZS5xdWFkMTtcbiAgaWYgKGlkeCA9PT0gMikgcmV0dXJuIG5vZGUucXVhZDI7XG4gIGlmIChpZHggPT09IDMpIHJldHVybiBub2RlLnF1YWQzO1xuICByZXR1cm4gbnVsbDtcbn1cblxuZnVuY3Rpb24gc2V0Q2hpbGQobm9kZSwgaWR4LCBjaGlsZCkge1xuICBpZiAoaWR4ID09PSAwKSBub2RlLnF1YWQwID0gY2hpbGQ7XG4gIGVsc2UgaWYgKGlkeCA9PT0gMSkgbm9kZS5xdWFkMSA9IGNoaWxkO1xuICBlbHNlIGlmIChpZHggPT09IDIpIG5vZGUucXVhZDIgPSBjaGlsZDtcbiAgZWxzZSBpZiAoaWR4ID09PSAzKSBub2RlLnF1YWQzID0gY2hpbGQ7XG59XG4iLCJtb2R1bGUuZXhwb3J0cy5tYWluID0gZnVuY3Rpb24gKCkge1xuXG4gIGlmICh0eXBlb2Ygd2luZG93ID09PSAndW5kZWZpbmVkJykge1xuICAgIGNvbnNvbGUubG9nKFwiSW4gTm9kZS5qc1wiKVxuICB9IGVsc2Uge1xuICAgIGNvbnNvbGUubG9nKFwiSW4gQnJvd3NlclwiKVxuICB9XG4gIHZhciBncmFwaCA9IHJlcXVpcmUoJ25ncmFwaC5ncmFwaCcpKCk7XG5cbiAgLy8gVE9ETzogU2VwYXJhdGUgdGhlIFVJIGZyb20gdGhlIGJ1c2luZXNzIGxvZ2ljXG4gIHZhciBjcmVhdGVOb2RlcyA9IHJlcXVpcmUoJy4vbGliL21vY2tfZGF0YS5qcycpKGdyYXBoKVxuICB2YXIgbGF5b3V0ID0gY3JlYXRlTGF5b3V0KGdyYXBoKTtcblxuLy9JTEFQOiAgdmFyIGNyZWF0ZVBpeGlHcmFwaGljcyA9IHJlcXVpcmUoJ25ncmFwaC5waXhpJyk7XG4vL0lMQVA6ICB2YXIgcGl4aUdyYXBoaWNzID0gY3JlYXRlUGl4aUdyYXBoaWNzKGdyYXBoLCBsYXlvdXQpO1xuICAvLyBzZXR1cCBvdXIgY3VzdG9tIGxvb2tpbmcgbm9kZXMgYW5kIGxpbmtzOlxuLy9JTEFQOiAgcGl4aUdyYXBoaWNzLmNyZWF0ZU5vZGVVSShyZXF1aXJlKCcuL2xpYi9jcmVhdGVOb2RlVUknKSlcbi8vSUxBUDogICAgLnJlbmRlck5vZGUocmVxdWlyZSgnLi9saWIvcmVuZGVyTm9kZScpKVxuLy9JTEFQOiAgICAuY3JlYXRlTGlua1VJKHJlcXVpcmUoJy4vbGliL2NyZWF0ZUxpbmtVSScpKVxuLy9JTEFQOiAgICAucmVuZGVyTGluayhyZXF1aXJlKCcuL2xpYi9yZW5kZXJMaW5rJykpO1xuXG4vL0lMQVA6ICBjb25zb2xlLmxvZyhwaXhpR3JhcGhpY3MuZ3JhcGhHcmFwaGljcylcbi8vSUxBUDogIHBpeGlHcmFwaGljcy5ncmFwaEdyYXBoaWNzLnNjYWxlLnggPSAwLjM1XG5cbi8vSUxBUDogIHBpeGlHcmFwaGljcy5ncmFwaEdyYXBoaWNzLnNjYWxlLnkgPSAwLjM1XG5cbiAgLy9sYXlvdXQgPSBwaXhpR3JhcGhpY3MubGF5b3V0O1xuICAvL3ZhciBub2RlID0gZ3JhcGguZ2V0Tm9kZSh0eCkgXG4gIC8vbGF5b3V0LnBpbk5vZGUobm9kZSx0cnVlKTtcblxuICAvLyBiZWdpbiBhbmltYXRpb24gbG9vcDpcbiAgY29uc29sZS5sb2coXCJBQUFBOlwiICsgSlNPTi5zdHJpbmdpZnkoY3JlYXRlTm9kZXMpKVxuICBjcmVhdGVOb2RlcygwKVxuLy9JTEFQOiAgcGl4aUdyYXBoaWNzLnJ1bigpO1xuXG59XG5cbmZ1bmN0aW9uIGNyZWF0ZUxheW91dChncmFwaCkge1xuICB2YXIgbGF5b3V0ID0gcmVxdWlyZSgnbmdyYXBoLmZvcmNlbGF5b3V0JyksXG4gICAgICBwaHlzaWNzID0gcmVxdWlyZSgnbmdyYXBoLnBoeXNpY3Muc2ltdWxhdG9yJyk7XG5cbiAgcmV0dXJuIGxheW91dChncmFwaCwgcGh5c2ljcyh7XG4gICAgICAgICAgc3ByaW5nTGVuZ3RoOiA4MCxcbiAgICAgICAgICBzcHJpbmdDb2VmZjogMC4wMDAyLFxuICAgICAgICAgIGRyYWdDb2VmZjogMC4wMDAyLFxuICAgICAgICAgIGdyYXZpdHk6IC0zMCxcbiAgICAgICAgICB0aGV0YTogMC43XG4gICAgICAgIH0pKTtcbn1cblxuIiwiLyoqXG4gKiBAZmlsZVxuICogUHJvdmlkZXMgcmFuZG9tIG5vZGVzIGFuZCBsaW5rcyB0aGF0IHNpbXVsYXRlcyB0cmFuc2FjdGlvbnNcbiAqXG4gKiBcbiAqL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIG5vZGVzIChncmFwaCkge1xuICBcbiAgdmFyIHJlc3VsdCA9IGZ1bmN0aW9uIGNyZWF0ZU5vZGUgKGlkKSB7XG4gIFxuICAgIGlmICggaWQgPT09IHVuZGVmaW5lZCkge1xuICAgICAgaWQgPSAwXG4gICAgfVxuICBcbiAgICB2YXIgaXMgPSBnZXRSYW5kb21FeHAoMSwgMTIsIDE2KVxuICAgIHZhciBvcyA9IGdldFJhbmRvbUV4cCgyLCAyNClcblxuICAgIHZhciB0eCA9IGdyYXBoLmFkZE5vZGUoJ3QnICsgaWQsIHsgdHlwZTogJ3QnLCB2YWx1ZTogZ2V0UmFuZG9tRXhwKDEwMCwgMTAwMDAwMCwgMTAwMCApIH0pXG5cbiAgICBjb25zb2xlLmxvZyhcIkluOiAlZCwgT3V0OiAlZCwgVHhzOiAlc1wiLCBpcywgb3MsIEpTT04uc3RyaW5naWZ5KHR4KSlcblxuICAgIHZhciBpbyA9IHt9XG4gICAgdmFyIHJuID0gMCBcbiAgICBmb3IgKCB2YXIgaSA9IDA7IGkgPCBpczsgaSsrKSB7XG4gICAgICBybiA9IGdldFJhbmRvbUV4cCgxMCwgMTAwMDAwMCwgNDAwKSAgIFxuICAgICAgaW8gPSBhZGROb2RlKGdyYXBoLCAnaScsIGkgKyBpZCwgcm4pICBcbiAgICAgIC8vIGlvID0gZ3JhcGguYWRkTm9kZSgnaScgKyBpICsgaWQsIHsgdHlwZTogJ2knLCB2YWx1ZTogZ2V0UmFuZG9tRXhwKDEwLCAxMDAwMDAwLCA0MDApIH0pXG4gICAgICBncmFwaC5hZGRMaW5rKGlvLmlkLCB0eC5pZClcbiAgICB9XG4gICAgZm9yICggdmFyIGkgPSAwOyBpIDwgb3M7IGkrKykge1xuICAgICAgaW8gPSBncmFwaC5hZGROb2RlKCdvJyArIGkgKyBpZCwgeyB0eXBlOiAnbycsIHZhbHVlOiBnZXRSYW5kb21FeHAoMTAsIDEwMDAwMDAsIDQwMCkgfSlcbiAgICAgIGdyYXBoLmFkZExpbmsodHguaWQsIGlvLmlkKVxuICAgIH1cbiAgXG4gICAgaWYgKGlkIDwgMTAwKSB7XG4gICAgICBzZXRUaW1lb3V0KGZ1bmN0aW9uICgpIHtcbiAgICAgICAgaWQrK1xuICAgICAgICBjcmVhdGVOb2RlKGlkLnRvU3RyaW5nKCkgKVxuICAgICAgfSwgZ2V0UmFuZG9tRXhwKDUwMCwgNTAwMCwgMTAwMCkpXG4gICAgfVxuICB9XG4gIFxuICByZXR1cm4gcmVzdWx0XG59XG5cbi8qKlxuICogQ29uc3RydWN0IGEgbm9kZSBiYXNlZCBvbiBpdHMgdHlwZVxuICovXG52YXIgYWRkTm9kZSA9IGZ1bmN0aW9uIChncmFwaCwgdHlwZSwgaWQsIHZhbHVlKSB7XG4gIG5vZGVJZCA9IHR5cGUgKyBpZFxuICBkYXRhID0geyB0eXBlOiB0eXBlLCB2YWx1ZTogdmFsdWUgfVxuICBjb25zb2xlLmxvZyhcIk5vZGUgSUQ6ICVzLCBkYXRhOiAlc1wiLCBub2RlSWQsIEpTT04uc3RyaW5naWZ5KGRhdGEpKVxuICByZXR1cm4gZ3JhcGguYWRkTm9kZShub2RlSWQsIGRhdGEpICAgIFxufVxuXG4vKipcbiAqIEdlbmVyYXRlcyBhIHJhbmRvbSBpbnRlZ2VyIGJldHdlZW4gbWluIGFuZCBtYXguXG4gKlxuICogSWYgbWVhbiBpcyBub3QgbnVsbCB0aGVuIGl0IGdlbmVyYXRlcyByYW5kb20gbnVtYmVyIGZyb20gdGhlIGV4cG9uZW50aWFsIGRpc3RyaWJ1dGlvbiB3aXRoIHRoZSBzcGVjaWZpZWQgbWVhblxuICogT3RoZXJ3aXNlIGl0IG9ubHkgZ2VuZXJhdGVzIGEgcmFuZG9tIG51bWJlciBiZXR3ZWVuIG1pbiBhbmQgbWF4IHZhbHVlLlxuICpcbiAqIEBwYXJhbSB7SW50ZWdlcn0gbWluXG4gKiAgIE1pbmltdW0gdmFsdWUgb2YgdGhlIHJhbmRvbSBudW1iZXIuXG4gKiBAcGFyYW0ge0ludGVnZXJ9IG1heCBcbiAqICAgTWF4aW11bSB2YWx1ZSBvZiB0aGUgcmFuZG9tIG51bWJlci5cbiAqIEBwYXJhbSB7SW50ZWdlcn0gbWVhbiBcbiAqICAgTWVhbiB2YWx1ZSBmb3IgdGhlIGV4cG9uZW50aW9uIHJhbmRvbSBudW1iZXIuXG4gKiBcbiAqIEByZXR1cm5cbiAqICAgdGhlIGdlbmVyYXRlZCByYW5kb20gbnVtYmVyIG9yIC0xIG9uIGFueSBlcnJvci5cbiAqL1xuZnVuY3Rpb24gZ2V0UmFuZG9tRXhwKG1pbiwgbWF4LCBtZWFuKSB7XG5cbiAgbWluID0gTWF0aC5jZWlsKG1pbik7XG4gIG1heCA9IE1hdGguZmxvb3IobWF4KTtcbiAgcmVzdWx0ID0gLTFcblxuICBpZiAodHlwZW9mIG1lYW4gPT09ICd1bmRlZmluZWQnKSB7XG4gICAgIHJlc3VsdCA9IE1hdGguZmxvb3IoTWF0aC5yYW5kb20oKSAqIChtYXggLSBtaW4pKSArIG1pbjsgXG4gIH0gZWxzZSB7XG4gICAgIHJlc3VsdCA9IC1NYXRoLmxvZyhNYXRoLnJhbmRvbSgpKSAqIG1lYW4gXG4gICAgIHJlc3VsdCA9IHJlc3VsdCA+IG1heCA/IG1heCA6IHJlc3VsdCA8PSBtaW4gPyBtaW4gOiByZXN1bHRcbiAgICAgcmVzdWx0ID0gTWF0aC5mbG9vcihyZXN1bHQpXG4gIH1cbiAgXG4gIHJldHVybiByZXN1bHRcbn1cbiJdfQ==
