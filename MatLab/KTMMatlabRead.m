classdef KTMMatlabRead
    properties
        hasDepth = 0
        depthWidth = 0
        depthHeight = 0
        
        hasRGB = 0
        RGBWidth = 0
        RGBHeight = 0
        EOF = 1
    end
    properties(SetAccess = private)
        file = 0
        dataStart = 0
        depthFrameDataSize = 0
        RGBFrameDataSize = 0
        totalFrameDataSize = 0
        frames = 0
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
            obj.depthFrameDataSize = (4 + 5 + (obj.depthWidth * obj.depthHeight * 2)) * obj.hasDepth;
            obj.RGBFrameDataSize = (4 + 5 + (obj.RGBWidth * obj.RGBHeight * 3)) * obj.hasRGB;
            obj.totalFrameDataSize = obj.depthFrameDataSize + obj.RGBFrameDataSize;;
            fseek(obj.file,0,'eof');
            dataEnd = ftell(obj.file);
            dataSize = dataEnd - obj.dataStart;
            obj.frames = dataSize / obj.totalFrameDataSize;
            fprintf('Total frames: %i\n', obj.frames);
            fseek(obj.file,obj.dataStart,'bof');
        end
        function [obj,outFrameTime,outDepthFrame,outRGBFrame] = nextFrame(obj)
            outDepthFrame = [];
            outRGBFrame = [];
            outFrameTime = 0;
            
            if(obj.hasDepth)
                frameType = fread(obj.file, 5, '*char').'
                outFrameTime = fread(obj.file,1,'long');
                
                outDepthFrame = zeros(obj.depthHeight, obj.depthWidth, 'uint16');
                for i=1:obj.depthHeight,
                    outDepthFrame(i,:) = fread(obj.file,obj.depthWidth,'uint16');
                end
            end
            if(obj.hasRGB)
                frameType = fread(obj.file, 5, '*char').'
                outFrameTime = fread(obj.file,1,'long');
                
                outRGBFrame = zeros(obj.RGBHeight, obj.RGBWidth, 3, 'uint8');
                data = fread(obj.file,obj.RGBHeight * obj.RGBWidth * 3,'uint8');
                for i=1:obj.RGBHeight,
                    start = 1 + ((i - 1) * obj.RGBWidth);
                    line = data(start:start + obj.RGBWidth - 1);
                    for j=1:obj.RGBWidth / 3,
                        %Conversion from BGR to RGB
                        BGR = line([j, j + 1, j + 2]);
                        outRGBFrame(i,j,:) = fliplr(BGR);
                    end
                end
            end
        end
        function [obj,positionSet] = seekFrame(obj, frameNumber)
            if(frameNumber > obj.frames || frameNumber < 1)
                positionSet = 0;
                return;
            end
            fseek(obj.file, obj.dataStart + (obj.totalFrameDataSize * (frameNumber - 1)), 'bof');
            positionSet = 1;
        end
    end
end