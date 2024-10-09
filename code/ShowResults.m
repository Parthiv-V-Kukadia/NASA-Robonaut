function ShowResults(filename)

% Load file with results
load(filename);

% Separate competitors by status
finished = struct('team',{},'time',{},'status',{},'controller',{});
crashed = struct('team',{},'time',{},'status',{},'controller',{});
for i=1:length(results)
    if (results(i).status == 1)
        finished(end+1) = results(i); %#ok<AGROW>
    elseif (results(i).status == 0)
        crashed(end+1) = results(i); %#ok<AGROW>
    else
        error(sprintf(['You should run ''ShowResults'' only when all\n' ...
                       'competitors have either crashed or finished.'])); %#ok<SPERR>
    end
end

% Sort finished competitors by time
%   http://blogs.mathworks.com/pick/2010/09/17/sorting-structure-arrays-based-on-fields/
if ~isempty(finished)
    Afields = fieldnames(finished);
    Acell = struct2cell(finished);
    sz = size(Acell);
    % Convert to a matrix
    Acell = reshape(Acell, sz(1), []);      % Px(MxN)
    % Make each field a column
    Acell = Acell';                         % (MxN)xP
    % Sort by second field "time"
    Acell = sortrows(Acell,2);
    % Put back into original cell array format
    Acell = reshape(Acell', sz);
    % Convert to Struct
    finished = cell2struct(Acell, Afields, 1);
end

% List of competitors that have finished
fprintf(1,'\nFINISHED:\n');
for i=1:length(finished)
    fprintf(1,'%10.2f : %10.10s : %s\n',finished(i).time,finished(i).team,finished(i).controller);
end
fprintf(1,'\n');

% List of competitors that have crashed
fprintf(1,'\nCRASHED:\n');
for i=1:length(crashed)
    fprintf(1,'%10.2f : %10.10s : %s\n',crashed(i).time,crashed(i).team,crashed(i).controller);
end
fprintf(1,'\n');

end