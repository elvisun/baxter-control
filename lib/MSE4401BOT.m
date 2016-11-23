classdef MSE4401BOT
    %MSE4401BOT Matlab control class for MSE4401 robots
    %   This class is used to interact with the robot used in the MSE4401 labs. 
    %   The MSE4401 Robot-Matlab Link program has to be running in the background.
    %   The order of joints in the arrays of data that are passed to or received
    %   from the functions in this class are as follows:
    %   [Base, Shoulder, Elbow, Roll, Pitch, Yaw, Gripper]
    properties (Access = private)
        hUDPSender
        hUDPReceiver
    end
    methods
        function MB = MSE4401BOT(outPort, inPort)
            %MSE4401BOT Creates a new MSE4401BOT object with the given ports
            %   Robot = MSE4401BOT(outPort, inPort) creates a new MSE4401BOT
            %   object that sends commands on the UDP port 'outPort' and 
            %   receives data on the UDP port 'inPort'. The values of 'outPort'
            %   and 'inPort' must match the port number entered in the 
            %   'UDP In Port' and 'UDP Out Port' fields respectively in the
            %   MSE4401 Robot-Matlab Link program. Port numbers can range from
            %   1024 to 65535 only, and 'outPort' and 'inPort' must be different.
            outPort = uint32(outPort);
            inPort = uint32(inPort);
            if ((outPort < 1024) || (outPort > 65535) || (inPort < 1024) || (inPort > 65535))
                throw(MException('Constructor:BadInput','Port numbers can range from 1024 to 65535 only'));  
            end
            if (outPort == inPort) 
                throw(MException('Constructor:BadInput','outPort and inPort must be different'));  
            end
            MB.hUDPSender = dsp.UDPSender('RemoteIPAddress','127.0.0.1','RemoteIPPort',uint16(outPort),'LocalIPPortSource','Auto');
            MB.hUDPReceiver = dsp.UDPReceiver('RemoteIPAddress','127.0.0.1','LocalIPPort',uint16(inPort),'MessageDataType','uint8','MaximumMessageLength',15,'ReceiveBufferSize',15);
            step(MB.hUDPReceiver);
        end
        function sendPosition(MB, position)
            %sendPosition Sends new joint positions to the Robot
            %   Robot.sendPosition(q) sends the new joint positions 'q' to the
            %   Robot and confirms that the MSE4401 Robot-Matlab Link program
            %   acknowledges the new positions. 'q' must be an array of 7 positions
            %   in degrees that can only range from 0 to 300 degrees. Trying to
            %   call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection, or if the Robot is connected but
            %   not enabled will result in a communication error.
            if length(position) ~= 7 || numel(position) ~= 7
                throw(MException('sendPosition:BadInput','Input must be an array of 7 positions in degrees'));  
            end
            for i=1:7
                if (position(i) < 0) || (position(i) > 300)
                    throw(MException('sendPosition:BadInput','Position can only range from 0 to 300 degrees'));  
                end
            end
            position = uint16(round(double(position)*10));
            packet = uint8(zeros(15,1));
            packet(1) = uint8(1);
            for i=1:7
                packet(i*2)     = uint8(bitand(position(i),255));
                packet(i*2 + 1) = uint8(bitand(uint8(floor(double(position(i))/256)),255));
            end
            step(MB.hUDPSender, packet);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 1
                throw(MException('sendPosition:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply ~= packet(1)
                throw(MException('sendPosition:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
        end
        function sendSpeed(MB, speed)
            %sendSpeed Sends new joint speeds to the Robot
            %   Robot.sendSpeed(v) sends the new joint speeds 'v' to the
            %   Robot and confirms that the MSE4401 Robot-Matlab Link program
            %   acknowledges the new speeds. 'v' must be an array of 7 speeds
            %   in rpm that can only range from 0.1 to 5 rpm. Trying to
            %   call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection, or if the Robot is connected but
            %   not enabled will result in a communication error.
            if length(speed) ~= 7 || numel(speed) ~= 7
                throw(MException('sendSpeed:BadInput','Input must be an array of 7 speeds in rpm'));  
            end
            for i=1:7
                if (speed(i) < 0.1) || (speed(i) > 5)
                    throw(MException('sendSpeed:BadInput','Speed can only range from 0.1 to 5 rpm'));  
                end
            end
            speed = uint16(round(double(speed)*10));
            packet = uint8(zeros(15,1));
            packet(1) = uint8(2);
            for i=1:7
                packet(i*2)     = uint8(bitand(speed(i),255));
                packet(i*2 + 1) = uint8(bitand(uint8(floor(double(speed(i))/256)),255));
            end
            step(MB.hUDPSender, packet);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 1
                throw(MException('sendSpeed:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply ~= packet(1)
                throw(MException('sendSpeed:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
        end
        function sendTorque(MB, torque)
            %sendTorque Sends new joint torque limits to the Robot
            %   Robot.sendTorque(t) sends the new joint torque limits 't' to the
            %   Robot and confirms that the MSE4401 Robot-Matlab Link program
            %   acknowledges the new torque limits. 't' must be an array of 7 torques
            %   in Nmm that can only range from 0 to 1700 Nmm. Trying to
            %   call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection, or if the Robot is connected but
            %   not enabled will result in a communication error.
            if length(torque) ~= 7 || numel(torque) ~= 7
                throw(MException('sendTorque:BadInput','Input must be an array of 7 torques in Nmm'));  
            end
            for i=1:7
                if (torque(i) < 0) || (torque(i) > 1700)
                    throw(MException('sendTorque:BadInput','Torque can only range from 0 to 1700 Nmm'));  
                end
            end
            torque = uint16(round(double(torque)));
            packet = uint8(zeros(15,1));
            packet(1) = uint8(3);
            for i=1:7
                packet(i*2)     = uint8(bitand(torque(i),255));
                packet(i*2 + 1) = uint8(bitand(uint8(floor(double(torque(i))/256)),255));
            end
            step(MB.hUDPSender, packet);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 1
                throw(MException('sendSpeed:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply ~= packet(1)
                throw(MException('sendSpeed:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
        end
        function position = getPosition(MB)
            %getPosition Gets the current joint positions from the Robot
            %   q = Robot.getPosition gets the current joint positions 'q' from the
            %   Robot. 'q' will be an array of 7 positions in degrees.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            command = uint8(1);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 15
                throw(MException('getPosition:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getPosition:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = uint16(reply);
            position = uint16(zeros(7,1));            
            for i=1:7
                position(i) = reply(i*2) + reply(i*2 + 1) * 256;
            end            
            position = double(position)/10.0;
        end
        function speed = getSpeed(MB)
            %getSpeed Gets the current joint speeds from the Robot
            %   v = Robot.getSpeed gets the current joint speeds 'v' from the
            %   Robot. 'v' will be an array of 7 speeds in rpm.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            command = uint8(2);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 15
                throw(MException('getSpeed:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getSpeed:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = int32(reply);
            speed = int32(zeros(7,1));   
            for i=1:7
                speed(i) = 0;
                if reply(i*2 + 1) > 127
                    speed(i) = -32768;
                end
				speed(i) = speed(i) + reply(i*2) + reply(i*2 + 1) * 256;
                if reply(i*2 + 1) > 127
                    speed(i) = speed(i)-32768;
                end
            end            
            speed = double(speed)/10.0;
        end
        function torque = getTorque(MB)
            %getTorque Gets the current joint torques from the Robot
            %   t = Robot.getTorque gets the current joint torques 't' from the
            %   Robot. 't' will be an array of 7 torques in Nmm.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            command = uint8(3);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 15
                throw(MException('getTorque:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getTorque:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = int32(reply);
            torque = int32(zeros(7,1));            
            for i=1:7
                torque(i) = 0;
                if reply(i*2 + 1) > 127
                    torque(i) = -32768;
                end
				torque(i) = torque(i) + reply(i*2) + reply(i*2 + 1) * 256;
                if reply(i*2 + 1) > 127
                    torque(i) = torque(i)-32768;
                end
            end            
            torque = double(torque);
        end   
        function voltage = getVoltage(MB)
            %getVoltage Gets the current joint servo voltages from the Robot
            %   V = Robot.getVoltage gets the current joint servo voltages 'V' from the
            %   Robot. 'V' will be an array of 7 voltages in Volts.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            command = uint8(4);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 8
                throw(MException('getVoltage:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getVoltage:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = uint8(reply);
            voltage = uint8(zeros(7,1));            
            for i=1:7
                voltage(i) = reply(i+1);
            end            
            voltage = double(voltage)/10.0;
        end 
        function temperature = getTemperature(MB)
            %getTemperature Gets the current joint servo temperatures from the Robot
            %   T = Robot.getTemperature gets the current joint servo temperatures 'T'
            %   from the Robot. 'T' will be an array of 7 temperatures in degrees Celsius.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            %   Be aware that the servos shut down due to overheating when the 
            %   temperature crosses about 74 degrees Celsius.
            command = uint8(5);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 8
                throw(MException('getTemperature:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getTemperature:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = uint8(reply);
            temperature = uint8(zeros(7,1));            
            for i=1:7
                temperature(i) = reply(i+1);
            end            
            temperature = double(temperature);
        end  
        function motion = getMotion(MB)
            %getMotion Gets the current joint motion status from the Robot
            %   m = Robot.getMotion gets the current joint motion status 'm'
            %   from the Robot. 'm' will be an array of 7 booleans, the value
            %   of which is true (1) if the corresponding joint is in motion.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            command = uint8(6);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 8
                throw(MException('getMotion:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getMotion:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = uint8(reply);
            motion = uint8(zeros(7,1));            
            for i=1:7
                motion(i) = reply(i+1);
            end            
            motion = boolean(motion);
        end 
        function error = getError(MB)
            %getError Gets the current joint servo error status from the Robot
            %   e = Robot.getError gets the current joint servo error status 'e'
            %   from the Robot. 'e' will be an array of 7 bytes, with each bit of
            %   the byte representing a specific error as follows: 
            %   Bit 6 - Instruction Error; Bit 5 - Overload; Bit 4 - Checksum Error
            %   Bit 3 - Range Error; Bit 2 - Overheat; Bit 1 - Angle Error
            %   Bit 0 - Voltage Error. The bit is set if the error has occurred.
            %   Trying to call this function when MSE4401 Robot-Matlab Link program has 
            %   not established UDP connection will result in a communication error.
            %   Calling tis function when the Robot is not connected will return zeros.
            command = uint8(7);
            step(MB.hUDPSender, command);
            tic;
            reply = step(MB.hUDPReceiver);
            while (toc < 0.055) && (length(reply) == 0)
                reply = step(MB.hUDPReceiver);
            end
            if length(reply) ~= 8
                throw(MException('getError:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            if reply(1) ~= command
                throw(MException('getError:ComFail','No communications from the MSE4401 Robot-Matlab Link program'));  
            end
            
            reply = uint8(reply);
            error = uint8(zeros(7,1));            
            for i=1:7
                error(i) = reply(i+1);
            end            
            error = uint8(error);
        end 
    end
end