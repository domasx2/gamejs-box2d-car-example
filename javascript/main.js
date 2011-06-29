var gamejs = require('gamejs');
var box2d = require('./box2d');
var utils = require('./utils');

var STEER_NONE=STEER_NONE=0;
var STEER_RIGHT=STEER_RIGHT=1;
var STEER_LEFT=STEER_LEFT=2;

var ACC_NONE=ACC_NONE=0;
var ACC_ACCELERATE=ACC_ACCELERATE=1;
var ACC_BRAKE=ACC_BRAKE=2;

var WIDTH_PX=800   //screen width in pixels
var HEIGHT_PX=600; //screen height in pixels
var SCALE=10;      //how many pixels in a meter
var WIDTH_M=WIDTH_PX/SCALE; //world width in meters. for this example, world is as large as the screen
var HEIGHT_M=HEIGHT_PX/SCALE; //world height in meters
var KEYS_DOWN={};
var b2world;

var font=new gamejs.font.Font('16px Sans-serif');

var BINDINGS={accelerate:gamejs.event.K_UP, 
                brake:gamejs.event.K_DOWN,      
                steer_left:gamejs.event.K_LEFT, 
                steer_right:gamejs.event.K_RIGHT}; 

function handleEvent(event){
    if (event.type === gamejs.event.KEY_DOWN) {
        KEYS_DOWN[event.key] = true;
    } else if (event.type === gamejs.event.KEY_UP) {
        KEYS_DOWN[event.key] = false;
    }; 
};


var BoxProp = function(pars){
    /*
    just a simple constructor for physical props
     
     pars:
     size - array [width, height]
     position - array [x, y], in world meters, of center
    */
    this.size=pars.size;
    var bdef=new box2d.b2BodyDef();
    bdef.position=new box2d.b2Vec2(pars.position[0], pars.position[1]);
    bdef.angle=0;
    bdef.fixedRotation=true;
    this.body=b2world.CreateBody(bdef);
    var sdef=new box2d.b2PolygonDef();
    sdef.SetAsBox(this.size[0]/2, this.size[1]/2);
    sdef.restitution=0.4; //positively bouncy!
    this.body.CreateShape(sdef);
    return this;  
};

function Wheel(pars){
    /*
    pars - object with attributes:
    
    car - car this wheel belongs to
    x - horizontal position in meters relative to car's center
    y - vertical position in meters relative to car's center
    width - width in meters
    length - length in meters
    revolving - does this wheel revolve when steering?
    powered - is this wheel powered?
    */

    this.position=[pars.x, pars.y];
    this.car=pars.car;
    this.revolving=pars.revolving;
    this.powered=pars.powered;

    //initialize physical body
    var def=new box2d.b2BodyDef();
    def.position=this.car.body.GetWorldPoint(new box2d.b2Vec2(this.position[0], this.position[1]));
    def.angle=this.car.body.GetAngle();
    this.body=b2world.CreateBody(def);
    this.body.SetMassFromShapes();
    var boxdef=new box2d.b2PolygonDef();
    boxdef.SetAsBox(pars.width/2, pars.length/2);
    boxdef.density=1;
    boxdef.isSensor=true; //wheel is not collidable. 'physical' wheels don't work well with this version of box2d :/
    this.body.CreateShape(boxdef);
    this.body.SetMassFromShapes();

    //create joint to connect wheel to body
    if(this.revolving){
        var jointdef=new box2d.b2RevoluteJointDef();
        jointdef.Initialize(this.car.body, this.body, this.body.GetWorldCenter());
        jointdef.enableMotor=false; //using motor to rotate the wheel does not work well for me with this box2d versio
    }else{
        var jointdef=new box2d.b2PrismaticJointDef();
        jointdef.Initialize(this.car.body, this.body, this.body.GetWorldCenter(), new box2d.b2Vec2(1, 0));
        jointdef.enableLimit=true;
        jointdef.lowerTranslation=jointdef.upperTranslation=0;
    }
    b2world.CreateJoint(jointdef);



}

Wheel.prototype.setAngle=function(angle){
    /*
    angle - wheel angle relative to car, in degrees
    */
    this.body.SetXForm(this.body.GetPosition(), this.car.body.GetAngle()+utils.radians(angle));
};

Wheel.prototype.getLocalVelocity=function(){
    /*returns get velocity vector relative to car
    */
    var res=this.car.body.GetLocalVector(this.car.body.GetLinearVelocityFromLocalPoint(new box2d.b2Vec2(this.position[0], this.position[1])));
    return [res.x, res.y];
};

Wheel.prototype.getDirectionVector=function(){
    /*
    returns a world unit vector pointing in the direction this wheel is moving
    */
    return utils.rotateVector( (this.getLocalVelocity().y>0) ? [0, 1]:[0, -1] , utils.degrees(this.body.GetAngle())) ;
};


Wheel.prototype.getKillVelocityVector=function(){
    /*
    substracts sideways velocity from this wheel's velocity vector and returns the remaining front-facing velocity vector
    */
    var velocity=this.body.GetLinearVelocity();
    var sideways_axis=this.getDirectionVector();
    var dotprod=utils.vectorDotProduct([velocity.x, velocity.y], sideways_axis);
    return [sideways_axis[0]*dotprod, sideways_axis[1]*dotprod];
};

Wheel.prototype.killSidewaysVelocity=function(){
    /*
    removes all sideways velocity from this wheels velocity
    */
    var kv=this.getKillVelocityVector();
    this.body.SetLinearVelocity(new box2d.b2Vec2(kv[0], kv[1]));

};


function Car(pars){
    /*
    pars is an object with possible attributes:\
    
    width - width of the car in meters
    length - length of the car in meters
    position - starting position of the car, array [x, y] in meters
    angle - starting angle of the car, degrees
    max_steer_angle - maximum angle the wheels turn when steering, degrees
    max_speed       - maximum speed of the car, km/h
    power - engine force, in newtons, that is applied to EACH powered wheel
    wheels - wheel definitions: [{x, y, rotatable, powered}}, ...] where
             x is wheel position in meters relative to car body center
             y is wheel position in meters relative to car body center
             revolving - boolean, does this turn rotate when steering?
             powered - is force applied to this wheel when accelerating/braking?
    */

    this.steer=STEER_NONE;
    this.accelerate=ACC_NONE;
    
    this.max_steer_angle=pars.max_steer_angle;
    this.max_speed=pars.max_speed;
    this.power=pars.power;
    this.wheel_angle=0;//keep track of current wheel angle relative to car
    
    //create physical body
    var def=new box2d.b2BodyDef();
    def.position=new box2d.b2Vec2(pars.position[0], pars.position[1]);
    def.angle=utils.radians(pars.angle); 
    def.linearDamping=0.15; 
    def.bullet=true; //dedicates more time to collision detection - car travelling at high speeds at low framerates otherwise might teleport through obstacles.
    def.angularDamping=0.3;
    this.body=b2world.CreateBody(def);
    var shapedef=new box2d.b2PolygonDef();
    shapedef.SetAsBox(pars.width/2, pars.length/2);
    shapedef.density=1;
    shapedef.friction=0.3;
    shapedef.restitution=0.4;
    this.body.CreateShape(shapedef);
    this.body.SetMassFromShapes();
    
    //initialize wheels
    this.wheels=[]
    var wheeldef, i;
    for(i=0;i<pars.wheels.length;i++){
        wheeldef=pars.wheels[i];
        wheeldef.car=this;
        this.wheels.push(new Wheel(wheeldef));
    }
}

Car.prototype.getPoweredWheels=function(){
    var retv=[];
    for(var i=0;i<this.wheels.length;i++){
        if(this.wheels[i].powered){
            retv.push(this.wheels[i]);
        }
    }
    return retv;
};

Car.prototype.getLocalVelocity=function(){
    /*
    returns car's velocity vector relative to the car
    */
    var retv=this.body.GetLocalVector(this.body.GetLinearVelocityFromLocalPoint(new box2d.b2Vec2(0, 0)));
    return [retv.x, retv.y];
};

Car.prototype.getRevolvingWheels=function(){
    var retv=[];
    for(var i=0;i<this.wheels.length;i++){
        if(this.wheels[i].revolving){
            retv.push(this.wheels[i]);
        }
    }
    return retv;
};

Car.prototype.getSpeedKMH=function(){
    var velocity=this.body.GetLinearVelocity();
    var len=utils.vectorLength([velocity.x, velocity.y]);
    return (len/1000)*3600;
};

Car.prototype.setSpeed=function(speed){
    /*
    speed - speed in kilometers per hour
    */
    var velocity=this.body.GetLinearVelocity();
    velocity=utils.normaliseVector([velocity.x, velocity.y]);
    velocity=new box2d.b2Vec2(velocity[0]*((speed*1000.0)/3600.0),
                              velocity[1]*((speed*1000.0)/3600.0));
    this.body.SetLinearVelocity(velocity);

};

Car.prototype.update=function(msDuration){
    
        //1. KILL SIDEWAYS VELOCITY
        
        //kill sideways velocity for all wheels
        var i;
        for(i=0;i<this.wheels.length;i++){
            this.wheels[i].killSidewaysVelocity();
        }
    
        //2. SET WHEEL ANGLE
  
        //calculate the change in wheel's angle for this update, assuming the wheel will reach is maximum angle from zero in 200 ms
        var incr=(this.max_steer_angle/200) * msDuration;
        
        if(this.steer==STEER_RIGHT){
            this.wheel_angle=Math.min(Math.max(this.wheel_angle, 0)+incr, this.max_steer_angle) //increment angle without going over max steer
        }else if(this.steer==STEER_LEFT){
            this.wheel_angle=Math.max(Math.min(this.wheel_angle, 0)-incr, -this.max_steer_angle) //decrement angle without going over max steer
        }else{
            this.wheel_angle=0;        
        }

        //update revolving wheels
        var wheels=this.getRevolvingWheels();
        for(i=0;i<wheels.length;i++){
            wheels[i].setAngle(this.wheel_angle);
        }
        
        //3. APPLY FORCE TO WHEELS
        var base_vect; //vector pointing in the direction force will be applied to a wheel ; relative to the wheel.
        
        //if accelerator is pressed down and speed limit has not been reached, go forwards
        if((this.accelerate==ACC_ACCELERATE) && (this.getSpeedKMH() < this.max_speed)){
            base_vect=[0, -1];
        }
        else if(this.accelerate==ACC_BRAKE){
            //braking, but still moving forwards - increased force
            if(this.getLocalVelocity()[1]<0)base_vect=[0, 1.3];
            //going in reverse - less force
            else base_vect=[0, 0.7];
        }
        else base_vect=[0, 0];

        //multiply by engine power, which gives us a force vector relative to the wheel
        var fvect=[this.power*base_vect[0], this.power*base_vect[1]];

        //apply force to each wheel
        wheels=this.getPoweredWheels();
        for(i=0;i<wheels.length;i++){
           var position=wheels[i].body.GetWorldCenter();
           wheels[i].body.ApplyForce(wheels[i].body.GetWorldVector(new box2d.b2Vec2(fvect[0], fvect[1])), position );
        }
    
        
        //if going very slow, stop - to prevent endless sliding
        if( (this.getSpeedKMH()<4) &&(this.accelerate==ACC_NONE)){
            this.setSpeed(0);
        }

};

function drawBody(display, body){
    /*
    Draw a body.
    TODO: upgrade to box2dweb, use debugdraw.
    */
    var shape=body.GetShapeList(); //returns a shape instead of list. Bug?
    var vertices=shape.m_vertices;
    var p, points=[];
    for(var i=0;i<vertices.length;i++){
        if(vertices[i]){
            p=body.GetWorldPoint(vertices[i]);
            points.push([p.x*SCALE, p.y*SCALE])
        }
    }
    gamejs.draw.polygon(display, '#000000', points, 1);
}

function main(){
    /*
    initialize everything, start game loop 
    */
    var display = gamejs.display.setMode([WIDTH_PX, HEIGHT_PX]);
    
    //SET UP B2WORLD
    var worldAABB=new box2d.b2AABB();
    //when an object crosses a bound (leaves the screen), it is frozen. make bounds a few meters beyond the screen, so parts of objects do not remain sticking out
    worldAABB.lowerBound=new box2d.b2Vec2(-10, -10);
    worldAABB.upperBound=new box2d.b2Vec2(WIDTH_M+10, HEIGHT_M+10);
    b2world=new box2d.b2World(worldAABB, new box2d.b2Vec2(0, 0), true);
    
    //initialize car
    var car=new Car({'width':4,
                    'length':8,
                    'position':[30, 20],
                    'angle':0,
                    'power':360,
                    'max_steer_angle':20,
                    'max_speed':100,
                    'wheels':[{'x':-2, 'y':-2.4, 'width':0.8, 'length':1.6, 'revolving':true, 'powered':true},
                                {'x':2, 'y':-2.4, 'width':0.8, 'length':1.6, 'revolving':true, 'powered':true},
                                {'x':-2, 'y':2.4, 'width':0.8, 'length':1.6, 'revolving':false, 'powered':false},
                                {'x':2, 'y':2.4, 'width':0.8, 'length':1.6, 'revolving':false, 'powered':false}]});
    
    //initialize some props to bounce against
    var props=[];
    props.push(new BoxProp({'size':[WIDTH_M, 1],    'position':[WIDTH_M/2, 0.5]}));
    props.push(new BoxProp({'size':[1, HEIGHT_M-2], 'position':[0.5, HEIGHT_M/2]}));
    props.push(new BoxProp({'size':[WIDTH_M, 1],    'position':[WIDTH_M/2, HEIGHT_M-0.5]}));
    props.push(new BoxProp({'size':[1, HEIGHT_M-2], 'position':[WIDTH_M-0.5, HEIGHT_M/2]}));
    
    var center=[WIDTH_M/2, HEIGHT_M/2];
    props.push(new BoxProp({'size':[1, 12], 'position':[center[0]-6, center[0]]}));
    props.push(new BoxProp({'size':[1, 12], 'position':[center[0]+6, center[0]]}));
    props.push(new BoxProp({'size':[11, 1], 'position':[center[0], center[0]+5.5]}));
    
    function tick(msDuration) {
        //GAME LOOP
        
        //handle events
        gamejs.event.get().forEach(handleEvent);
        
        //set car controls according to player input
        if(KEYS_DOWN[BINDINGS.accelerate]){
            car.accelerate=ACC_ACCELERATE;
        }else if(KEYS_DOWN[BINDINGS.brake]){
            car.accelerate=ACC_BRAKE;
        }else{
            car.accelerate=ACC_NONE;
        }
        
        if(KEYS_DOWN[BINDINGS.steer_right]){
            car.steer=STEER_RIGHT;
        }else if(KEYS_DOWN[BINDINGS.steer_left]){
            car.steer=STEER_LEFT;
        }else{
            car.steer=STEER_NONE;
        }
        
        //update physics world
        b2world.Step(msDuration/1000, 10, 8);        
        
        //update car
        car.update(msDuration);
        
        //fill background
        gamejs.draw.rect(display, '#FFFFFF', new gamejs.Rect([0, 0], [WIDTH_PX, HEIGHT_PX]),0)
        
        var i;
        //draw car and its wheels
        drawBody(display, car.body);
        for(i=0;i<car.wheels.length;i++){
            drawBody(display, car.wheels[i].body);
        }
        
        //draw props
        for(i=0; i<props.length;i++){
            drawBody(display, props[i].body);
        }
        
        //fps and car speed
        display.blit(font.render('FPS: '+parseInt((1000)/msDuration)), [15, 15]);
        display.blit(font.render('SPEED: '+parseInt(Math.ceil(car.getSpeedKMH()))+' km/h'), [15, 45]);
        return;
    };
    gamejs.time.fpsCallback(tick, this, 60);
    
}

gamejs.ready(main);
