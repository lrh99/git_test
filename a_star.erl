%%----------------------------------------------------
%%%	File:		a_star.erl
%%%	Author:		liurh@163.com
%%%	Created:	2017.3.17
%%% Updated:    2017.3.17
%%%	Purpose:	a星算法
%%----------------------------------------------------

-module(a_star).
-export([
            find_path_list/5, cal_map_node_amount/4, read_map/1
        ]).

%% 节点数据结构
%% {当前节点的id(从1算起), 当前点在迷宫中的x坐标(从1算起, 代表第几行),
%%  当前点在迷宫中的y坐标(从1算起, 代表第几列),起始点到当前点实际代价,
%%  当前节点到目标节点最佳路径的估计代价,起始点到目标点的估计代价(f = g + h)
%%  当前节点的父节点id}
-record(a_star_struct ,{id, x, y, g, h, f, father_id}).

-define(NODE_EMPTY, 0).             %% 可以通过的节点
-define(NODE_OBSTACLE, 1).          %% 障碍物，不可通过
-define(START_NODE_ID, 1).          %% 起始点id
-define(START_NODE_FATHER_ID, 0).   %% 起始点的父节点id
-define(DIRECT_AMOUNT, 8).          %% 当前点可移动的方向数量
-define(MOVE_FUNCS, 
    [
        fun move_left/2, fun move_top/2, fun move_right/2, 
        fun move_down/2, fun move_top_left/2, fun move_top_right/2, 
        fun move_down_right/2, fun move_down_left/2
    ]).                             %% 移动函数列表, 从向左开始, 顺时针共8个方向
%%----------------------------------------------------

%% 寻找路径
%% in:
%% 1.NodeWidth: 节点宽度(以像素为单位)
%% 2.NodeHigh: 节点高度(以像素为单位)
%% 3.StartPixelX: 起始点X坐标(以像素为单位)
%% 4.StartPixelY: 起始点Y坐标(以像素为单位)
%% 5.EndPixelX: 结束点X坐标(以像素为单位)
%% 6.EndPixelY: 结束点Y坐标(以像素为单位)
%% 7.Rows: 地图节点行数
%% 8.Columns: 地图节点列数
%% 9.MazeLsLs: 二维地图的节点是否有障碍物, MazeLsLs = [[0,1,...],...]
%% out: 
%% 1. {error, _}
%% 2. {ok, PixelCoordLs}, PixelCoordLs = [{PixelX,PixelY},...]
%%    PixelX: 以像素为单位的X坐标, PixelY: 以像素为单位的Y坐标
find_path_list([NodeWidth, NodeHigh], [Rows, Columns], MazeLsLs, [StartPixelX, StartPixelY], [EndPixelX, EndPixelY]) -> 
    {StartX, StartY} = convert_pixel_to_node_coord(NodeWidth, NodeHigh,
        StartPixelX, StartPixelY),
    {EndX, EndY} = convert_pixel_to_node_coord(NodeWidth, NodeHigh, 
        EndPixelX, EndPixelY),    
    
    InitRet = init(StartX, StartY, EndX, EndY, Rows, Columns, MazeLsLs),
    {StartNode, EndNode, {Rows, Columns, MazeLsLs}} = InitRet,

    case find_path(StartNode, EndNode, Rows, Columns, MazeLsLs) of 
        {error, _} = FindPathErr -> FindPathErr;
        {ok, CoordLs} -> 
            PixelCoordLs = 
                lists:map(fun({NodeX, NodeY}) -> 
                    convert_node_coord_to_pixel(NodeWidth, NodeHigh, NodeX, NodeY)
                end, CoordLs),
            {ok, PixelCoordLs}
    end.

%%----------------------------------------------------
%% 根据地图分辨率、格子分辨率计算格子数量
%% out: integer()
cal_map_node_amount(Width, High, NodeWidth, NodeHigh) -> 
    MapArea = Width * High,
    NodeArea = NodeWidth * NodeHigh,
    MapArea div NodeArea.
    
%% 把分辨率坐标转换成节点坐标
%% out: 
%% {NodeX,NodeY}, NodeX: X方向格子坐标, NodeY: Y方向格子坐标
convert_pixel_to_node_coord(NodeWidth, NodeHigh, PixelX, PixelY) -> 
    %% 计算X方向上的格子数
    NodeX = cal_node_coord(NodeWidth, PixelX),
    %% 计算Y方向上的格子数
    NodeY = cal_node_coord(NodeHigh, PixelY),
    {NodeX, NodeY}.
        
%% 根据给定的像素计算格子数
cal_node_coord(Distance, Pixel) -> 
    if Pixel == 0 -> 1;
       true -> 
            Div = Pixel div Distance,
            Rem = Pixel rem Distance,
            if Rem > 0 -> Div + 1;
               true -> Div
            end
    end.    
    
%% 把节点坐标转换成分辨率坐标  
%% out: 
%% {PixelX,PixelY}, PixelX: X方向像素坐标, PixelY: Y方向像素坐标
convert_node_coord_to_pixel(NodeWidth, NodeHigh, NodeX, NodeY) -> 
    %% 计算的是格子(节点)左上方坐标故减1
    PixelX = (NodeX - 1) * NodeWidth,
    PixelY = (NodeY - 1) * NodeHigh,
    {PixelX, PixelY}.

%% 寻找路径
%% out: 
%% 1. {error, _}
%% 2. {ok, CoordLs}: 生成的从起始点到目标点的坐标列表, CoordLs = [{X,Y},...]
find_path(StartNode, EndNode, Rows, Columns, MazeLsLs) 
  when is_record(StartNode, a_star_struct) andalso is_record(EndNode, a_star_struct) -> 
    if StartNode#a_star_struct.x =:= EndNode#a_star_struct.x andalso StartNode#a_star_struct.y =:= EndNode#a_star_struct.y -> 
            {ok, [{StartNode#a_star_struct.x, StartNode#a_star_struct.y}]};
       true -> 
            %% 将开始节点放入开放列表
            InitOpenedTable = [StartNode],
            InitClosedTable = [],
            BeginIdCounter = StartNode#a_star_struct.id,
            %% 循环查找
            LoopRet = loop_find_node(InitOpenedTable, InitClosedTable, EndNode, Rows, Columns, MazeLsLs, BeginIdCounter),
            case LoopRet of 
                {error, _} = LoopFindErr -> 
                    LoopFindErr;
                {ok, {ClosedTable, FinalEndNode}} -> 
                    generate_path(ClosedTable, FinalEndNode)                    
            end
    end.
    
%% 根据封闭列表,生成路径
%% out: 
%% 1. {error, _}
%% 2. {ok, CoordLs}: 生成的从起始点到目标点的坐标列表, CoordLs = [{X,Y},...]
generate_path(ClosedTable, EndNode) -> 
    EndCoord = {EndNode#a_star_struct.x, EndNode#a_star_struct.y},
    InitCoordLs = [EndCoord],
    EndNodeFatherId = EndNode#a_star_struct.father_id,
    
    CoordLs = 
    while(fun({NodeLs, AccCoordLs, FatherId}) -> 
        if  FatherId =:= ?START_NODE_FATHER_ID -> {false, AccCoordLs};
            true -> 
                FindRet = lists:keytake(FatherId, 2, NodeLs),
                case FindRet of 
                    false -> 
                        {false, {error, <<"Error_PathNotContinue">>}};
                    {value, FatherNode, RemainNodeLs} -> 
                        Coord = {FatherNode#a_star_struct.x, FatherNode#a_star_struct.y},
                        NewCoordLs = [Coord | AccCoordLs],
                        NewFatherId = FatherNode#a_star_struct.father_id,
                        {true, {RemainNodeLs, NewCoordLs, NewFatherId}}
                end 
        end
    end, {ClosedTable, InitCoordLs, EndNodeFatherId}),
    
    case CoordLs of 
        {error, _} = PathErr -> PathErr;
        _ -> {ok, CoordLs}
    end. 
    
%% 循环查找最优节点
%% out: 
%% 1. {error, _}
%% 2. {ok, {ClosedTable,EndNode}}
%%  ClosedTable: 封闭列表, 列表中每个元素都是#a_star_struct
%%  EndNode: 目标节点, 并存有其父id
loop_find_node([], _, _ ,_, _, _, _) -> 
    {error, <<"Error_OpenTableEmpty">>};
loop_find_node(OpenedTable, ClosedTable, EndNode, Rows, Columns, MazeLsLs, IdCounter) -> 
    AscOpenedTable = 
    lists:sort(fun(Node1, Node2) -> 
        if Node1#a_star_struct.f =< Node2#a_star_struct.f -> true;
           true -> false
        end
    end, OpenedTable),

    [CurrentNode | OpenedTableTail] = AscOpenedTable,
    %% 判断是否已经搜寻到目标节点
    if CurrentNode#a_star_struct.x =:= EndNode#a_star_struct.x andalso CurrentNode#a_star_struct.y =:= EndNode#a_star_struct.y -> 
            {ok, {ClosedTable, CurrentNode}};
        true -> 
            %% 把当前节点加入到封闭列表
            NewClosedTable = [CurrentNode | ClosedTable],
            %% 求有效相邻坐标
            ValidCoordLs = 
            lists:foldl(fun(MoveFun, AccValidCoordLs) -> 
                {NextX, NextY} = Coord = MoveFun(CurrentNode#a_star_struct.x, CurrentNode#a_star_struct.y),
                IsCoordValid = is_valid(NextX, NextY, CurrentNode#a_star_struct.x, CurrentNode#a_star_struct.y, Rows, Columns, MazeLsLs),
                if not IsCoordValid -> AccValidCoordLs;
                   true -> 
                        IsInClosedTalbe = 
                            lists:any(fun(Node) -> 
                                if Node#a_star_struct.x =:= NextX andalso Node#a_star_struct.y =:= NextY -> 
                                        true;
                                   true -> false
                                end
                            end, NewClosedTable),
                        if IsInClosedTalbe -> AccValidCoordLs;
                           true -> [Coord | AccValidCoordLs]
                        end   
                end
            end, [], ?MOVE_FUNCS),

            %% 更新开放列表
            {NewOpenedTable, NewIdCounter} = 
            lists:foldl(fun({NextX, NextY}, {AccOpenTable, AccIdCounter}) -> 
                {SameNode, RemainOpenTable} = 
                while(fun({NodeLs, RemainNodeLs}) -> 
                    if NodeLs =:= [] -> 
                            {false, {none, RemainNodeLs}};
                        true -> 
                            [Node | NodeLsTail] = NodeLs,
                            if Node#a_star_struct.x =:= NextX
                                andalso
                               Node#a_star_struct.y =:= NextY -> 
                                    RemainNodeLs1 = RemainNodeLs ++ NodeLsTail,
                                    {false, {Node, RemainNodeLs1}};
                               true -> 
                                    RemainNodeLs1 = [Node | RemainNodeLs],
                                    {true, {NodeLsTail, RemainNodeLs1}}
                            end
                    end
                end, {AccOpenTable, []}),
                
                %% 计算该相邻节点的g值
                IsOnDiagonal = is_next_node_on_diagonal(NextX, NextY, CurrentNode#a_star_struct.x, CurrentNode#a_star_struct.y),
                NextG = 
                if IsOnDiagonal -> 
                        %% 该相邻节点与当前节点在对角线上
                        CurrentNode#a_star_struct.g + 14;
                   true -> 
                        CurrentNode#a_star_struct.g + 10
                end,
                
                case SameNode of 
                    none -> 
                        %% 该相邻节点不在开放列表中
                        NextId = AccIdCounter + 1,
                        NextH = manhattan_judge(EndNode#a_star_struct.x, EndNode#a_star_struct.y, NextX, NextY),
                        NewNode = #a_star_struct{id = NextId, x = NextX, y = NextY, 
                                g = NextG, h = NextH, f = NextG + NextH, father_id = CurrentNode#a_star_struct.id},
                        {[NewNode | RemainOpenTable], NextId};
                    _ -> 
                        %% 该相邻节点在开放列表中                        
                        if NextG < SameNode#a_star_struct.g -> 
                                SameNodeH = SameNode#a_star_struct.h,
                                BestNode = SameNode#a_star_struct{g = NextG, 
                                    f = NextG + SameNodeH, 
                                    father_id = CurrentNode#a_star_struct.id},
                                {[BestNode | RemainOpenTable], AccIdCounter};
                            true -> 
                                {[SameNode | RemainOpenTable], AccIdCounter}
                        end
                end
            end, {OpenedTableTail, IdCounter}, ValidCoordLs),
            
            loop_find_node(NewOpenedTable, NewClosedTable, EndNode, Rows, Columns, MazeLsLs, NewIdCounter)
    end.

%% 初始化数据
%% out: 
%% {StartNode, EndNode, {Rows,Columns,MazeLsLs}}
%%  StartNode: 开始节点, 数据结构参考#a_star_struct
%%  EndNode: 目标节点, 数据结构参考#a_star_struct 
%%  Rows: 与fun read_map/1的返回Rows值一致, Columns: 与fun read_map/1的返回Columns值一致
%%  MazeLsLs: 与fun read_map/1的返回MazeLsLs值一致
init(StartX, StartY, EndX, EndY, Rows, Columns, MazeLsLs) -> 
    %% 起始点
    StartG = 0,
    StartH = manhattan_judge(EndX, EndY, StartX, StartY),
    StartNode = #a_star_struct{id = ?START_NODE_ID, x = StartX, 
        y = StartY, g = StartG, h = StartH, f = StartG + StartH, father_id = ?START_NODE_FATHER_ID},
    EndNode = #a_star_struct{id = -1, x = EndX, y = EndY, g = -1, h = -1, f = -1, father_id = -1},
    {StartNode, EndNode, {Rows, Columns, MazeLsLs}}.

%% 读取a星地图配置数据
%% in: Filename = list()
%% out: 
%% 1. {error,_}
%% 2. {ok, Tup}
%%  Tup = {{Rows,Columns},{StartX,StartY,EndX,EndY},MazeLsLs}
%%  Rows: 行数, Columns: 列数, StartX: 起始点X坐标, StartY: 起始点Y坐标
%%  EndX: 目标点X坐标, EndY: 目标点Y坐标
%%  MazeLsLs: 二维地图的节点是否有障碍物, MazeLsLs = [[0,1,...],...]
read_map(Filename) -> 
    ReadTerm = file:consult(Filename),
    case ReadTerm of 
        {ok, [Tup]} -> 
            {{Rows, Columns},{_StartX, _StartY, _EndX, _EndY}, MazeLsLs} = Tup,
            if length(MazeLsLs) =/= Rows -> {error, <<"Error_RowNotMatch">>};
               true -> 
                    IsColNotMatch = 
                    lists:any(fun(Row) -> 
                        if length(Row) =/= Columns -> true;
                           true -> false
                        end
                    end, MazeLsLs),
                    if IsColNotMatch -> {error, <<"Error_ColumnNotMatch">>};
                       true -> {ok, Tup}
                    end
            end;            
        {error, _} = ReadMapErr -> ReadMapErr
    end. 
    
%% 曼哈顿方法
%% 估计当前点到目的点路径代价h
manhattan_judge(EndX, EndY, X, Y) -> 
    DisX = erlang:abs(EndX - X),
    DisY = erlang:abs(EndY - Y),
    (DisX + DisY) * 10.
    
%% 检查节点是否有效
is_valid(NextX, NextY, CurrX, CurrY, Rows, Columns, MazeLsLs) ->     
    if NextX < 1 -> false;
       NextX > Rows -> false;
       NextY < 1 -> false;
       NextY > Columns -> false;
       true -> 
            NextRow = lists:nth(NextX, MazeLsLs),
            NextNodeType = lists:nth(NextY, NextRow),
            if NextNodeType =:= ?NODE_OBSTACLE -> false;
               true -> 
                    %% 考虑对角方向不能直接穿过障碍物
                    IsOnDiagonal = is_next_node_on_diagonal(NextX, NextY, CurrX, CurrY),
                    if IsOnDiagonal -> 
                            %% 下一节点是在对角线
                            CurrRow = lists:nth(CurrX, MazeLsLs),
                            NodeType1 = lists:nth(NextY, CurrRow),
                            NodeType2 = lists:nth(CurrY, NextRow),
                            if NodeType1 =:= ?NODE_OBSTACLE -> false;
                               NodeType2 =:= ?NODE_OBSTACLE -> false;
                               true -> true
                            end;
                        true -> 
                            %% 下一节点不是对角
                            true
                    end
            end
    end.
    
%% 节点是否位于对角线上
is_next_node_on_diagonal(NextX, NextY, CurrX, CurrY) -> 
    DisX = abs(NextX - CurrX),
    DisY = abs(NextY - CurrY),
    if  DisX =:= 1 andalso DisY =:= 1 -> true;
        true -> false
    end.
    
%% while循环
while(Fun, Data) ->
    case Fun(Data) of
        {true, Data1}  ->  while(Fun, Data1);
        {false, Data1} -> Data1
    end.    

%%----------------------------------------------------
%% 移动函数定义
move_left(X, Y) -> 
    {X-1, Y}.
    
move_top(X, Y) -> 
    {X, Y+1}.
    
move_right(X, Y) -> 
    {X+1, Y}.

move_down(X, Y) -> 
    {X, Y-1}.
    
move_top_left(X, Y) -> 
    {X-1, Y+1}.

move_top_right(X, Y) -> 
    {X+1, Y+1}.
    
move_down_right(X, Y) -> 
    {X+1, Y-1}.
    
move_down_left(X, Y) -> 
    {X-1, Y-1}.










