local softbody = {}
softbody.__index = softbody
local ffi = require("ffi")

ffi.cdef("typedef struct {double x,y,   vx,vy,  fx,fy,  mass;} massPoint;")
ffi.cdef("typedef struct {int ix,iy,ix2,iy2; double dist;} joint;")
local ct = ffi.typeof("massPoint")
local jointtype = ffi.typeof("joint")
function softbody:new(x,y,dist,cx,cy,stiffness,damping,joints,useJoints)
        local body = setmetatable({
            x = x,
            y = y,
            damping = damping,
            stiffness = stiffness,
            mode = useJoints,
        },softbody)
    
    body.dist = dist
    body.pointCountX = cx
    body.pointCountY = cy
    body.points = {}
    for px = 0,body.pointCountX-1 do
        body.points[px] = {}
        for py = 0,body.pointCountY-1 do
            body.points[px][py] = ct(px*dist, py*dist,0,0,0,0,1)
        end
    end
    body.jc = joints
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
    for x = 0,self.pointCountX-1 do
        for y = 0,self.pointCountY-1 do
            
            for n = 0,self.jc do
                for m = 0,self.jc do
                    if self.points[x+n] and self.points[x+n][y+m] and (n ~= 0 or m ~= 0) and self.points[x][y]~=self.points[x+n][y+m] then
                        local d = math.sqrt(lensq(self.points[x][y].x,self.points[x][y].y,self.points[x+n][y+m].x,self.points[x+n][y+m].y))
--                        self.joints[#self.joints+1] = jointtype(self.points[x][y],self.points[x+n][y+m],d)
                          self.joints[#self.joints+1] = jointtype(x,y,x+n,y+m,d)
  
                    end
                  end
            end
            
            for n = 1,self.jc do
                for m = 1,self.jc do
                    if self.points[x-n] and self.points[x-n][y+m] and self.points[x][y]~=self.points[x-n][y+m] then
                        local d = math.sqrt(lensq(self.points[x][y].x,self.points[x][y].y,self.points[x-n][y+m].x,self.points[x-n][y+m].y))
--                        self.joints[#self.joints+1] = jointtype(self.points[x][y],self.points[x-n][y+m],d)
                          self.joints[#self.joints+1] = jointtype(x,y,x-n,y+m,d)
  
  end
                end
            end
        end
    end

--for i,v in ipairs(self.joints) do io.write(i,tostring(v.p1),tostring(v.p2),"\n") end   
--   print(#self.joints)
end









function softbody:draw()
    love.graphics.setPointSize(1)
    love.graphics.push()
    love.graphics.translate(self.x,self.y)
  --  self:drawJoints()
    self:drawOutline()
    for x = 0,self.pointCountX-1 do
        for y = 0,self.pointCountY-1 do
--            love.graphics.circle("line",self.points[x][y].x,self.points[x][y].y,self.dist/4)

            love.graphics.points(self.points[x][y].x,self.points[x][y].y)
        end
    end
    
    love.graphics.setPointSize(1)
    
    love.graphics.pop()
    
end




function softbody:drawJoints()
  love.graphics.push()
  love.graphics.translate(self.x,self.y)
    for i = 1,#self.joints do
      local jt = self.joints[i]
      --local p1 = self.joints[i][1]
      --  local p2 = self.joints[i][2]
        local p1 = self.points[jt.ix][jt.iy]
        local p2 = self.points[jt.ix2][jt.iy2]
        
      love.graphics.line(p1.x,p1.y,p2.x,p2.y)
    end 
    love.graphics.pop()
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
        return true
    end
    
 return true
end

function softbody:testCollision(dt)
    currentBody = self
    for x = 0,self.pointCountX-1 do
        for y = 0,self.pointCountY-1 do
             currentPoint  = self.points[x][y]
            world:queryBoundingBox(currentPoint.x+self.x,currentPoint.y+self.y,currentPoint.x+self.x+1,currentPoint.y+self.y+1,cb)
        end
    end
end

function softbody:selfCollision(dt)
  for px = 0,self.pointCountX-1 do
    for py = 0,self.pointCountY-1 do
      
      local p1 = self.points[px][py]                
        
        for px2 = px,self.pointCountX-1 do
          for py2 = py,self.pointCountY-1 do
            if px ~= px2 or py ~= py2 then
            local p2 = self.points[px2][py2]
      
            local edx = self.dist*(px2-px)
            local edy = self.dist*(py2-py)
            
            local expDist = math.sqrt(edx*edx+edy*edy)
        
            local dx,dy = p2.x-p1.x,p2.y-p1.y
            local n = math.sqrt(dx*dx+dy*dy)
            dx = dx/n
            dy = dy/n
        
            if n > 0.1 then
            
            local force = self.stiffness*(expDist - n)
            
            local dvx = p2.vx-p1.vx
            local dvy = p2.vy-p1.vy 
            
            local damp = self.damping*dotProduct(dx,dy,dvx,dvy)
              force = (force - damp)/n
              p1.fx = p1.fx - force*dx*dt
              p1.fy = p1.fy - force*dy*dt
            
              p2.fx = p2.fx + force*dx*dt
              p2.fy = p2.fy + force*dy*dt
            end
            end
          end
        end
      end
  end
end




local maxForce = 1e5

function softbody:calcJoints(dt)
    
    for i = 1,#self.joints do
        local jt = self.joints[i]

---        local p1,p2 = jt.p1,jt.p2
        local p1 = self.points[jt.ix][jt.iy]
        local p2 = self.points[jt.ix2][jt.iy2]
        
        local dx,dy = p2.x-p1.x,p2.y-p1.y
        local n = math.sqrt(dx*dx+dy*dy)
        dx = dx/n
        dy = dy/n
        
        if n > 0 then
            local force = self.stiffness*(jt.dist - n)
            local dvx = p1.vx-p2.vx
            local dvy = p1.vy-p2.vy 

            local damp = self.damping*dotProduct(dx,dy,dvx,dvy)
            force = (force + damp)/jt.dist
            p1.fx = p1.fx - force*dx*dt
            p1.fy = p1.fy - force*dy*dt
            
            p2.fx = p2.fx + force*dx*dt
            p2.fy = p2.fy + force*dy*dt
        end
    end
end


function softbody:calcBBox()
    local xmin,ymin = math.huge,math.huge
    local xmax,ymax =-math.huge,-math.huge
    
    for x = 0,self.pointCountX-1 do
        for y = 0,self.pointCountY-1 do
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
    for x = 0,self.pointCountX-1 do
        for y = 0,self.pointCountY-1 do
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
    for px = 0,self.pointCountX-1 do
        for py = 0,self.pointCountY-1 do
            self.points[px][py].vx = self.points[px][py].vx + self.points[px][py].fx*dt+gx*dt
            self.points[px][py].vy = self.points[px][py].vy + self.points[px][py].fy*dt+gy*dt
            self.points[px][py].fx = 0 
            self.points[px][py].fy = 0
        end
    end
end
function softbody:move(dt)
    for px = 0,self.pointCountX-1 do
      for py = 0,self.pointCountY-1 do
            self.points[px][py].x = self.points[px][py].x+self.points[px][py].vx*dt
            self.points[px][py].y = self.points[px][py].y+self.points[px][py].vy*dt
      end
    end
end
function softbody:drawOutline()
    for x = 0,self.pointCountX-2 do
        love.graphics.line(self.points[x][0].x,self.points[x][0].y,self.points[x+1][0].x,self.points[x+1][0].y)
    end  
    for x = 0,self.pointCountX-2 do
        love.graphics.line(self.points[x][self.pointCountY-1].x,self.points[x][self.pointCountY-1].y,self.points[x+1][self.pointCountY-1].x,self.points[x+1][self.pointCountY-1].y)
    end  
    for y = 0,self.pointCountY-2 do
        love.graphics.line(self.points[0][y].x,self.points[0][y].y,self.points[0][y+1].x,self.points[0][y+1].y)
    end
    for y = 0,self.pointCountY-2 do
        love.graphics.line(self.points[self.pointCountX-1][y].x,self.points[self.pointCountX-1][y].y,self.points[self.pointCountX-1][y+1].x,self.points[self.pointCountX-1][y+1].y)
    end
end

function softbody:update(dt)
    if not self.mode then
      self:selfCollision(dt)
    else
      self:calcJoints(dt)
    end
    self:applyForce(dt)
    self:move(dt)
    
    self:testCollision(dt)
    
    self.xmin,self.ymin,self.xmax,self.ymax = self:calcBBox()
    self:reCalibrate(self.xmin,self.ymin)    
end


return softbody