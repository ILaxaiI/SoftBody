local phy = love.physics
local softBody = require("softBody2")
DRAW = true
world = phy.newWorld(0,100)
local body = softBody:new(10,-600,4,200,200,4,7000,60)
for i = 1,50 do
    local b = phy.newBody(world,love.math.random()*800,love.math.random()*600+400)
    b:setAngle(love.math.random()*math.pi*2)
    local s = phy.newRectangleShape(love.math.random()*200+10,love.math.random()*200+10)
    local f = phy.newFixture(b,s)
end
local bdys = {}

for i = 1,10 do 
  --  bdys[i] = softBody:new(i*80,0,7,10,10,10,1e2,30)
    
end

local function drawBack(fix)
    local shape = fix:getShape()
    local body = fix:getBody()
    if shape:typeOf("CircleShape") then
        local cx, cy = body:getWorldPoints(shape:getPoint())
        love.graphics.circle("line", cx, cy, shape:getRadius())
    elseif shape:typeOf("PolygonShape") then
        love.graphics.polygon("line", body:getWorldPoints(shape:getPoints()))
    else
        love.graphics.line(body:getWorldPoints(shape:getPoints()))
    end
    return true
end

function love.draw()
    
    local xmin,ymin,xmax,ymax = body:getBBox()
    local gw,gh = love.graphics.getDimensions()
    love.graphics.push()
   -- love.graphics.translate(-xmin - (xmax-xmin)/2+gw/2,-ymin -(ymax-ymin)/2+gh/2)
    for i = 1,10 do 
        --bdys[i]:draw()
    end
    
    
    if DRAW then
    --body:drawJoints()
    
  
   end
   body:draw()
   world:queryBoundingBox(0,0,gw,gh,drawBack)--xmin + (xmax-xmin)/2-gw/2,ymin +(ymax-ymin)/2-gh/2,xmin + (xmax-xmin)/2+gw/2,ymin +(ymax-ymin)/2+gh/2,drawBack)

    love.graphics.setColor(1,0,0)
    love.graphics.setColor(1,1,1)
        
    love.graphics.rectangle("line",xmin,ymin,(xmax-xmin),(ymax-ymin))
    
    love.graphics.pop()
    love.graphics.print(love.timer.getFPS())
end

function love.keypressed(key)
    if key == "space" then
        DRAW = not DRAW
    end
end
--local mx,my = love.mouse.getPosition()
--local mbody = love.physics.newBody(world,mx,my,"dynamic")
--love.physics.newFixture(mbody,love.physics.newCircleShape(70))

--local mjoint = love.physics.newMouseJoint(mbody,mx,my)

function love.mousepressed(x,y)
    local xmin,ymin,xmax,ymax = body:getBBox()
    body:moveTo(x-(xmax-xmin)/2,y-(ymax-ymin)/2)

end


function love.update(dt)
    dt = math.min(dt,1/60)
    body:update(dt)

    world:update(dt)
  -- mjoint:setTarget(love.mouse.getPosition())
  
    for i = 1,10 do 
        --bdys[i]:update(dt)
    end
end

