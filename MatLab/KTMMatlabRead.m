classdef KTMMatlabRead
    properties
        hasDepth = 0
        depthWidth = 0
        depthHeight = 0
        
        hasRGB = 0
        RGBWidth = 0
        RGBHeight = 0
        EOF = 1
        currentFrame = 1
        frames = 0
    end
    properties(SetAccess = private)
        file = 0
        dataStart = 0
        frameInfoDataSize = 4 + 5
        depthFrameDataSize = 0
        RGBFrameDataSize = 0
        totalFrameDataSize = 0
    end
    methods
        function obj = openFile(obj, filePath)
            obj.hasDepth = 0;
            obj.depthWidth = 0;
            obj.depthHeight = 0;

            obj.hasRGB = 0;
            obj.RGBWidth = 0;
            obj.RGBHeight = 0;
            %'C:\Users\ganterd\Desktop\Dissertation\KTMKinectRecorder\output\saved.bin'
            obj.file = fopen(filePath);
            fileInfo = fread(obj.file,5,'*char').';
            obj.hasDepth = strcmp(fileInfo, 'DEPTH');
            if(obj.hasDepth)
                fileInfo = fread(obj.file,8,'*char').';
                if(strcmp(fileInfo,'80X60   '))
                    obj.depthWidth = 80;
                    obj.depthHeight = 60;
                elseif(strcmp(fileInfo,'320X240 '))
                    obj.depthWidth = 320;
                    obj.depthHeight = 240;
                elseif(strcmp(fileInfo,'640X480 '))
                    obj.depthWidth = 640;
                    obj.depthHeight = 480;
                end
                fprintf('Depth stream: %i x %i\n', obj.depthWidth, obj.depthHeight)
            else
                fseek(obj.file,-5,'cof');
                fprintf('Has no Depth Stream\n')
            end


            fileInfo = fread(obj.file,5,'*char').';
            obj.hasRGB = strcmp(fileInfo, 'RGB  ');
            if(obj.hasRGB)
                fileInfo = fread(obj.file,8,'*char').';
                if(strcmp(fileInfo,'640X480 '))
                    obj.RGBWidth = 640;
                    obj.RGBHeight = 480;
                elseif(strcmp(fileInfo,'1280X960'))
                    obj.RGBWidth = 1290;
                    obj.RGBHeight = 960;
                end
                fprintf('RGB stream: %i x %i\n', obj.RGBWidth, obj.RGBHeight)
            else
                fseek(obj.file,-5,'cof');
                fprintf('Has no RGB stream.\n')
            end
            
            obj.dataStart = ftell(obj.file);
            obj.depthFrameDataSize = ((obj.depthWidth * obj.depthHeight * 2)) * obj.hasDepth;
            obj.RGBFrameDataSize = ((obj.RGBWidth * obj.RGBHeight * 3)) * obj.hasRGB;
            obj.totalFrameDataSize = (obj.frameInfoDataSize * 2) + obj.depthFrameDataSize + obj.RGBFrameDataSize;;
            fseek(obj.file,0,'eof');
            dataEnd = ftell(obj.file);
            dataSize = dataEnd - obj.dataStart;
            obj.frames = dataSize / obj.totalFrameDataSize;
            fprintf('Total frames: %i\n', obj.frames);
            fseek(obj.file,obj.dataStart,'bof');
            obj.currentFrame = 1;
            obj.EOF = feof(obj.file);
        end
        function [obj,outFrameTime,outDepthFrame,outRGBFrame] = nextFrame(obj)
            outDepthFrame = [];
            outRGBFrame = [];
            outFrameTime = 0;
            
            if(obj.hasDepth)
                frameType = fread(obj.file, 5, '*char').'; % Not Used (Assuming File is Correct. I know. This is a horrible assumption, but short on time)
                outFrameTime = fread(obj.file,1,'long');
                
                data = fread(obj.file, obj.depthWidth * obj.depthHeight, 'uint16');
                outDepthFrame = uint16(reshape(data,obj.depthWidth,obj.depthHeight).');
            end
            if(obj.hasRGB)
                frameType = fread(obj.file, 5, '*char').'; % Not Used (Assuming File is Correct. I know. This is a horrible assumption, but short on time)
                outFrameTime = fread(obj.file,1,'long');
                
                data = fread(obj.file, obj.RGBFrameDataSize, 'uint8');
                organizedData = reshape(data,3,obj.RGBWidth,obj.RGBHeight);
                organizedData = uint8(permute(organizedData,[3 2 1]));
                outRGBFrame = uint8(flipdim(organizedData,3));
            end
            
            obj.EOF = feof(obj.file);
            obj.currentFrame = obj.currentFrame + 1;
        end
        function [obj,positionSet] = seekFrame(obj, frameNumber)
            if(frameNumber > obj.frames || frameNumber < 1)
                positionSet = 0;
                return;
            end
            fseek(obj.file, obj.dataStart + (obj.totalFrameDataSize * (frameNumber - 1)), 'bof');
            obj.currentFrame = 1;
            positionSet = 1;
        end
    end
end