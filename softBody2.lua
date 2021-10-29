local softbody = {}
softbody.__index = softbody
local ffi = require("ffi")

ffi.cdef("typedef struct {double x,y,   vx,vy,  fx,fy,  mass;} massPoint;")
ffi.cdef("typedef struct {massPoint* p1,p2; double dist;} joint;")
local ct = ffi.typeof("massPoint")
local jointtype = ffi.typeof("joint")
function softbody:new(x,y,dist,cx,cy,joints,stiffness,damping)
        local body = setmetatable({
            x = x,
            y = y,
            damping = damping,
            stiffness = stiffness,
            jc = joints,
            
        },softbody)
    
    body.dist = dist
    body.pointCountX = cx
    body.pointCountY = cy
    body.points = {}
    for px = 1,body.pointCountX do
        body.points[px] = {}
        for py = 1,body.pointCountY do
            body.points[px][py] = ct(px*dist, py*dist,0,0,0,0,1)
        end
    end
    
    body.joints = {}
    body:setUpJoints()   
    body.xmin,body.ymin,body.xmax,body.ymax = body:calcBBox()
    return body
end

local function lensq(x1,y1,x2,y2)
    local a,b = (x2-x1),(y2-y1)
    return a*a+b*b
end
function softbody:setUpJoints()
   
  local t1 = love.timer.getTime()
    for x = 1,self.pointCountX do
        for y = 1,self.pointCountY do
            
            for n = 0,self.jc do
                for m = 0,self.jc do
                    if self.points[x+n] and self.points[x+n][y+m] and (n ~= 0 or m ~= 0) and self.points[x][y]~=self.points[x+n][y+m] then
                        local d = math.sqrt(lensq(self.points[x][y].x,self.points[x][y].y,self.points[x+n][y+m].x,self.points[x+n][y+m].y))
                        self.joints[#self.joints+1] = jointtype(self.points[x][y],self.points[x+n][y+m],d)
                    end
                 end
            end
            
            for n = 1,self.jc do
                for m = 1,self.jc do
                    if self.points[x-n] and self.points[x-n][y+m] and self.points[x][y]~=self.points[x-n][y+m] then
                        local d = math.sqrt(lensq(self.points[x][y].x,self.points[x][y].y,self.points[x-n][y+m].x,self.points[x-n][y+m].y))
                        self.joints[#self.joints+1] = jointtype(self.points[x][y],self.points[x-n][y+m],d)
                    end
                end
            end
        end
    end
   
   
end










function softbody:draw()
    love.graphics.setPointSize(1)
    love.graphics.push()
    love.graphics.translate(self.x,self.y)
  --  self:drawJoints()
    self:drawOutline()
    for x = 1,self.pointCountX do
        for y = 1,self.pointCountY do
            love.graphics.points(self.points[x][y].x,self.points[x][y].y)
        end
    end
    
    love.graphics.setPointSize(1)
    
    love.graphics.pop()
    
end




function softbody:drawJoints()
    for i = 1,#self.joints do
        local p1 = self.joints[i][1]
        local p2 = self.joints[i][2]
        love.graphics.line(p1.x,p1.y,p2.x,p2.y)
    end 
end




local function getNormal(dx,dy)
    return dy,-dx
end

local function findIntersect(l1p1x,l1p1y, l1p2x,l1p2y, l2p1x,l2p1y, l2p2x,l2p2y)
	local a1,b1,a2,b2 = l1p2y-l1p1y, l1p1x-l1p2x, l2p2y-l2p1y, l2p1x-l2p2x
	local c1,c2 = a1*l1p1x+b1*l1p1y, a2*l2p1x+b2*l2p1y
	local det,x,y = a1*b2 - a2*b1
	if det==0 then return false end
	return (b2*c1-b1*c2)/det, (a1*c2-a2*c1)/det
end
local currentPoint
local currentBody

local function dotProduct(x1,y1,x2,y2)
    return x1*x2+y1*y2
end


local function cb(fix)
    local shape = fix:getShape()
    
    
    local Px,Py,Nx,Ny
    local l = math.huge
    
    local body = fix:getBody()
    if shape:typeOf("PolygonShape") then
        if not fix:testPoint(currentPoint.x+currentBody.x,currentPoint.y+currentBody.y) then return true end
    local fixp = {body:getWorldPoints(shape:getPoints())}

    for i = 1,7,2 do
        local x1,y1 = fixp[i],fixp[i+1]
        local x2,y2 = fixp[(i+1)%8+1],fixp[(i+2)%8+1]

        local nx,ny = getNormal(x2-x1,y2-y1)
        local nl = math.sqrt(nx*nx+ny*ny)
        nx = nx / nl
        ny = ny / nl
 
        local px,py = findIntersect(currentPoint.x+currentBody.x,currentPoint.y+currentBody.y,currentPoint.x+currentBody.x+nx,currentPoint.y+currentBody.y+ny,x1,y1,x2,y2)
        local dx,dy = px-(currentPoint.x+currentBody.x),py-(currentPoint.y+currentBody.y)
        local l2 = math.sqrt(dx*dx+dy*dy)
        if l2 < l then
            l = l2
            Px,Py = px,py
            Nx,Ny = nx,ny
        end
    end
    
    elseif shape:typeOf("CircleShape") then
        local x1,y1 = body:getPosition()
        local r = shape:getRadius()
        Nx,Ny = x1 - (currentPoint.x+currentBody.x),y1 - (currentPoint.y+currentBody.y)
        local nl = math.sqrt(Nx*Nx+Ny*Ny)
        if nl < r then
        Nx = -Nx/nl
        Ny = -Ny/nl
        Px = Nx*(r)+x1
        Py = Ny*(r)+y1
        end
        
    end
    if Px then
        if body:getType() == "dynamic" then
        end
        
        currentPoint.x = Px-currentBody.x+Nx*0.001
        currentPoint.y = Py-currentBody.y+Ny*0.001
        local d = dotProduct(Nx,Ny,currentPoint.vx,currentPoint.vy)
        currentPoint.vx = (currentPoint.vx - 2*d*Nx)
        currentPoint.vy = (currentPoint.vy - 2*d*Ny)
        return false
    end
    
 return true
end

function softbody:testCollision(dt)
    currentBody = self
    for x = 1,self.pointCountX do
        for y = 1,self.pointCountY do
             currentPoint  = self.points[x][y]
            world:queryBoundingBox(currentPoint.x+self.x,currentPoint.y+self.y,currentPoint.x+self.x+1,currentPoint.y+self.y+1,cb)
        end
    end
    

end

local maxForce = 1e5

ffi.cdef([[
void calcJoints(joint* J){
    p1 = &J.p1;
    p1.mass = 100;
};]])

function softbody:calcJoints(dt)
    
    for i = 1,#self.joints do
        
        ffi.C.calcJoints(self.joints[i])
        local jt = self.joints[i]

        local p1,p2 = jt.p1,jt.p2
        print(p1.mass)
       
        local dx,dy = p2.x-p1.x,p2.y-p1.y
        local n = math.sqrt(dx*dx+dy*dy)    
        dx = dx/n
        dy = dy/n
        
        if n > 0 then
            local force = self.stiffness*(jt.dist - n)/(jt.dist*self.jc)
            local dvx = p1.vx-p2.vx
            local dvy = p1.vy-p2.vy 

            local damp = self.damping*dotProduct(dx,dy,dvx,dvy)/(jt.dist*self.jc)

            force = (force + damp)
            p1.fx = p1.fx - force*dx
            p1.fy = p1.fy - force*dy


            p2.fx = p2.fx + force*dx
            p2.fy = p2.fy + force*dy
        end
    end
    
    
end


function softbody:calcBBox()
    local xmin,ymin = math.huge,math.huge
    local xmax,ymax =-math.huge,-math.huge
    
    for x = 1,self.pointCountX do
        for y = 1,self.pointCountY do
            xmin  = math.min(self.points[x][y].x,xmin)
            ymin  = math.min(self.points[x][y].y,ymin)
            xmax  = math.max(self.points[x][y].x,xmax)
            ymax  = math.max(self.points[x][y].y,ymax)
        end
    end
    return xmin,ymin,xmax,ymax
end
function softbody:getBBox()
    return self.xmin+self.x,self.ymin+self.y,self.xmax+self.x,self.ymax+self.y
end

function softbody:reCalibrate(xmin,ymin)
    
    self.x = self.x + xmin
    self.y = self.y + ymin
    for x = 1,self.pointCountX do
        for y = 1,self.pointCountY do
        self.points[x][y].x = self.points[x][y].x - xmin
        self.points[x][y].y = self.points[x][y].y - ymin
        end
    end
end


function softbody:moveTo(x,y)
    local dx = x-self.x
    local dy = y-self.y
    self.x = x
    self.y = y
end


function softbody:applyForce(dt)
    local gx,gy = world:getGravity()
    for px = 1,self.pointCountX do
        for py = 1,self.pointCountY do
            self.points[px][py].vx = self.points[px][py].vx + self.points[px][py].fx*dt +gx*dt
            self.points[px][py].vy = self.points[px][py].vy + self.points[px][py].fy*dt +gy*dt
 
 
            self.points[px][py].fx = 0 
            self.points[px][py].fy = 0
        end
    end
end
function softbody:move(dt)
    for px = 1,self.pointCountX do
        for py = 1,self.pointCountY do
            self.points[px][py].vx = math.min(70,math.max(-70,self.points[px][py].vx))
            self.points[px][py].vy = math.min(70,math.max(-70,self.points[px][py].vy))
            
            self.points[px][py].x = self.points[px][py].x+self.points[px][py].vx*dt
            self.points[px][py].y = self.points[px][py].y+self.points[px][py].vy*dt
        end
    end
end
function softbody:drawOutline()
    for x = 1,self.pointCountX-1 do
        love.graphics.line(self.points[x][1].x,self.points[x][1].y,self.points[x+1][1].x,self.points[x+1][1].y)
    end  
    for x = 1,self.pointCountX-1 do
        love.graphics.line(self.points[x][self.pointCountY].x,self.points[x][self.pointCountY].y,self.points[x+1][self.pointCountY].x,self.points[x+1][self.pointCountY].y)
    end  
    for y = 1,self.pointCountY-1 do
        love.graphics.line(self.points[1][y].x,self.points[1][y].y,self.points[1][y+1].x,self.points[1][y+1].y)
    end
    for y = 1,self.pointCountY-1 do
        love.graphics.line(self.points[self.pointCountX][y].x,self.points[self.pointCountX][y].y,self.points[self.pointCountX][y+1].x,self.points[self.pointCountX][y+1].y)
    end
end

function softbody:update(dt)
    self:calcJoints(dt)
    self:applyForce(dt)
self:move(dt)
    self:testCollision(dt)
    
    
    self.xmin,self.ymin,self.xmax,self.ymax = self:calcBBox()
    self:reCalibrate(self.xmin,self.ymin)
    
end


return softbody