%CLASS_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef Freenect2 < handle
    properties (SetAccess = private, Hidden = true)
        context;
        device;
        frames;
        mat;
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = Freenect2(varargin)
            this.context = freenect2_matlabd('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            freenect2_matlabd('delete', this.context, this.device);
        end

        %% enumerateDevices - an example class method call
        function device_count = enumerateDevices(this, varargin)
            device_count = freenect2_matlabd('enumerateDevices', this.context, varargin{:});
        end

        %% openDefaultDevice - another example class method call
        function device = openDevice(this, varargin)
            
            numvarargs = length(varargin);
            if numvarargs > 2
                error('Freenect2:openDevice:TooManyInputs', ...
                    'requires at most 2 optional inputs');
            end
            
            optargs = {'' PipelineType.CPU};
            optargs(1:numvarargs) = varargin;
            [serial, type] = optargs{:};
            
            this.device = freenect2_matlabd('openDevice', this.context, serial, uint8(type));
            device = this.device;
        end
        
        %% initializeDevice - another example class method call
        function initializeDevice(this, varargin)
            freenect2_matlabd('initializeDevice', this.context, this.device, varargin{:});
        end
        
        %% getFrame - another example class method call
        function frames = processFrame(this, varargin)
            this.frames = freenect2_matlabd('processFrame', this.context, this.device, varargin{:});
            frames = this.frames;
        end
        
        %% getFrameAsMatrix - another example class method call
        function frame = getFrame(this, type, varargin)
            frame = freenect2_matlabd('getFrame', this.context, this.device, uint8(type), varargin{:});
            frame = permute(frame, [3 2 1]);
        end
        
                %% getFrame - another example class method call
        function cleanupFrame(this, varargin)
            freenect2_matlabd('cleanupFrame', this.context, this.device, varargin{:});
        end
        
    end
end