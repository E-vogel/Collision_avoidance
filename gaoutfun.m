function [state,options,optchanged] = gaoutfun(options,state,flag)
    persistent state_record 
    if isempty(state_record)
      state_record = struct('Population', {},'BestScoreIndex',{});
    end
    if nargin == 0
      state = state_record;
      options = [];
      optchanged = [];
    else
      BestScore = min(state.Score);
      BestScoreIndex = find(state.Score == BestScore);
      state_record(end+1) = struct('Population', state.Population,'BestScoreIndex', BestScoreIndex);
      optchanged = false;
    end