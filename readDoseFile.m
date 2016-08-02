function [Data,plotInfo,NumBlocks] = readDoseFile(path, type)
% READDOSEFILE M-file
% 
%**************************************************************************
fid = fopen(path); % Change accordingly (With Metal)
numplots=15;
% read column headers
Intro = textscan(fid,'%s',11,'Delimiter','\n');
 
Block = 1; 

while (~feof(fid))                               % For each block:

  % fprintf('Block: %s\n', num2str(Block));           % Print block number to the screen
   InputText = textscan(fid,'%s',5,'delimiter','\n');  % Read 5 header lines
   HeaderLines{Block,1} = InputText{1};
  % disp(HeaderLines{Block});                        % Display header lines

    plotinfo =HeaderLines{Block,1}{2};
    plotInfo{Block,1} =plotinfo(20:end);
   
        LineText=fgetl(fid);
        LineText = cell2mat(textscan(LineText,'%f %f %f', 'CollectOutput',true'));
       
        n=1;
        while (~feof(fid)) 
           dat(n,1) = LineText(1);
           dat(n,2) = LineText(2);
           dat(n,3) = LineText(3);
           LineText=fgetl(fid);
           if strcmp(num2str(LineText),'&')~=1  
              LineText = cell2mat(textscan(LineText,'%f %f %f', 'CollectOutput',true')); 
              n=n+1;
           else
              break
           end
           
        end
        
        Data{Block,1} = dat;
        [NumRows,NumCols] = size(Data{Block});           % Determine size of table
                                                 % New line
   Block = Block+1;                                 % Increment block index
end

fclose(fid);
NumBlocks = Block-1;


