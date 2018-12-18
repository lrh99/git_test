%%----------------------------------------------------
%%%	File:		a_star.erl
%%%	Author:		liurh@163.com
%%%	Created:	2017.3.17
%%% Updated:    2017.3.17
%%%	Purpose:	a���㷨
%%----------------------------------------------------

-module(a_star).
-export([
            find_path_list/5, cal_map_node_amount/4, read_map/1
        ]).

%% �ڵ����ݽṹ
%% {��ǰ�ڵ��id(��1����), ��ǰ�����Թ��е�x����(��1����, ����ڼ���),
%%  ��ǰ�����Թ��е�y����(��1����, ����ڼ���),��ʼ�㵽��ǰ��ʵ�ʴ���,
%%  ��ǰ�ڵ㵽Ŀ��ڵ����·���Ĺ��ƴ���,��ʼ�㵽Ŀ���Ĺ��ƴ���(f = g + h)
%%  ��ǰ�ڵ�ĸ��ڵ�id}
-record(a_star_struct ,{id, x, y, g, h, f, father_id}).

-define(NODE_EMPTY, 0).             %% ����ͨ���Ľڵ�
-define(NODE_OBSTACLE, 1).          %% �ϰ������ͨ��
-define(START_NODE_ID, 1).          %% ��ʼ��id
-define(START_NODE_FATHER_ID, 0).   %% ��ʼ��ĸ��ڵ�id
-define(DIRECT_AMOUNT, 8).          %% ��ǰ����ƶ��ķ�������
-define(MOVE_FUNCS, 
    [
        fun move_left/2, fun move_top/2, fun move_right/2, 
        fun move_down/2, fun move_top_left/2, fun move_top_right/2, 
        fun move_down_right/2, fun move_down_left/2
    ]).                             %% �ƶ������б�, ������ʼ, ˳ʱ�빲8������
%%----------------------------------------------------

%% Ѱ��·��
%% in:
%% 1.NodeWidth: �ڵ���(������Ϊ��λ)
%% 2.NodeHigh: �ڵ�߶�(������Ϊ��λ)
%% 3.StartPixelX: ��ʼ��X����(������Ϊ��λ)
%% 4.StartPixelY: ��ʼ��Y����(������Ϊ��λ)
%% 5.EndPixelX: ������X����(������Ϊ��λ)
%% 6.EndPixelY: ������Y����(������Ϊ��λ)
%% 7.Rows: ��ͼ�ڵ�����
%% 8.Columns: ��ͼ�ڵ�����
%% 9.MazeLsLs: ��ά��ͼ�Ľڵ��Ƿ����ϰ���, MazeLsLs = [[0,1,...],...]
%% out: 
%% 1. {error, _}
%% 2. {ok, PixelCoordLs}, PixelCoordLs = [{PixelX,PixelY},...]
%%    PixelX: ������Ϊ��λ��X����, PixelY: ������Ϊ��λ��Y����
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
%% ���ݵ�ͼ�ֱ��ʡ����ӷֱ��ʼ����������
%% out: integer()
cal_map_node_amount(Width, High, NodeWidth, NodeHigh) -> 
    MapArea = Width * High,
    NodeArea = NodeWidth * NodeHigh,
    MapArea div NodeArea.
    
%% �ѷֱ�������ת���ɽڵ�����
%% out: 
%% {NodeX,NodeY}, NodeX: X�����������, NodeY: Y�����������
convert_pixel_to_node_coord(NodeWidth, NodeHigh, PixelX, PixelY) -> 
    %% ����X�����ϵĸ�����
    NodeX = cal_node_coord(NodeWidth, PixelX),
    %% ����Y�����ϵĸ�����
    NodeY = cal_node_coord(NodeHigh, PixelY),
    {NodeX, NodeY}.
        
%% ���ݸ��������ؼ��������
cal_node_coord(Distance, Pixel) -> 
    if Pixel == 0 -> 1;
       true -> 
            Div = Pixel div Distance,
            Rem = Pixel rem Distance,
            if Rem > 0 -> Div + 1;
               true -> Div
            end
    end.    
    
%% �ѽڵ�����ת���ɷֱ�������  
%% out: 
%% {PixelX,PixelY}, PixelX: X������������, PixelY: Y������������
convert_node_coord_to_pixel(NodeWidth, NodeHigh, NodeX, NodeY) -> 
    %% ������Ǹ���(�ڵ�)���Ϸ�����ʼ�1
    PixelX = (NodeX - 1) * NodeWidth,
    PixelY = (NodeY - 1) * NodeHigh,
    {PixelX, PixelY}.

%% Ѱ��·��
%% out: 
%% 1. {error, _}
%% 2. {ok, CoordLs}: ���ɵĴ���ʼ�㵽Ŀ���������б�, CoordLs = [{X,Y},...]
find_path(StartNode, EndNode, Rows, Columns, MazeLsLs) 
  when is_record(StartNode, a_star_struct) andalso is_record(EndNode, a_star_struct) -> 
    if StartNode#a_star_struct.x =:= EndNode#a_star_struct.x andalso StartNode#a_star_struct.y =:= EndNode#a_star_struct.y -> 
            {ok, [{StartNode#a_star_struct.x, StartNode#a_star_struct.y}]};
       true -> 
            %% ����ʼ�ڵ���뿪���б�
            InitOpenedTable = [StartNode],
            InitClosedTable = [],
            BeginIdCounter = StartNode#a_star_struct.id,
            %% ѭ������
            LoopRet = loop_find_node(InitOpenedTable, InitClosedTable, EndNode, Rows, Columns, MazeLsLs, BeginIdCounter),
            case LoopRet of 
                {error, _} = LoopFindErr -> 
                    LoopFindErr;
                {ok, {ClosedTable, FinalEndNode}} -> 
                    generate_path(ClosedTable, FinalEndNode)                    
            end
    end.
    
%% ���ݷ���б�,����·��
%% out: 
%% 1. {error, _}
%% 2. {ok, CoordLs}: ���ɵĴ���ʼ�㵽Ŀ���������б�, CoordLs = [{X,Y},...]
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
    
%% ѭ���������Žڵ�
%% out: 
%% 1. {error, _}
%% 2. {ok, {ClosedTable,EndNode}}
%%  ClosedTable: ����б�, �б���ÿ��Ԫ�ض���#a_star_struct
%%  EndNode: Ŀ��ڵ�, �������丸id
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
    %% �ж��Ƿ��Ѿ���Ѱ��Ŀ��ڵ�
    if CurrentNode#a_star_struct.x =:= EndNode#a_star_struct.x andalso CurrentNode#a_star_struct.y =:= EndNode#a_star_struct.y -> 
            {ok, {ClosedTable, CurrentNode}};
        true -> 
            %% �ѵ�ǰ�ڵ���뵽����б�
            NewClosedTable = [CurrentNode | ClosedTable],
            %% ����Ч��������
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

            %% ���¿����б�
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
                
                %% ��������ڽڵ��gֵ
                IsOnDiagonal = is_next_node_on_diagonal(NextX, NextY, CurrentNode#a_star_struct.x, CurrentNode#a_star_struct.y),
                NextG = 
                if IsOnDiagonal -> 
                        %% �����ڽڵ��뵱ǰ�ڵ��ڶԽ�����
                        CurrentNode#a_star_struct.g + 14;
                   true -> 
                        CurrentNode#a_star_struct.g + 10
                end,
                
                case SameNode of 
                    none -> 
                        %% �����ڽڵ㲻�ڿ����б���
                        NextId = AccIdCounter + 1,
                        NextH = manhattan_judge(EndNode#a_star_struct.x, EndNode#a_star_struct.y, NextX, NextY),
                        NewNode = #a_star_struct{id = NextId, x = NextX, y = NextY, 
                                g = NextG, h = NextH, f = NextG + NextH, father_id = CurrentNode#a_star_struct.id},
                        {[NewNode | RemainOpenTable], NextId};
                    _ -> 
                        %% �����ڽڵ��ڿ����б���                        
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

%% ��ʼ������
%% out: 
%% {StartNode, EndNode, {Rows,Columns,MazeLsLs}}
%%  StartNode: ��ʼ�ڵ�, ���ݽṹ�ο�#a_star_struct
%%  EndNode: Ŀ��ڵ�, ���ݽṹ�ο�#a_star_struct 
%%  Rows: ��fun read_map/1�ķ���Rowsֵһ��, Columns: ��fun read_map/1�ķ���Columnsֵһ��
%%  MazeLsLs: ��fun read_map/1�ķ���MazeLsLsֵһ��
init(StartX, StartY, EndX, EndY, Rows, Columns, MazeLsLs) -> 
    %% ��ʼ��
    StartG = 0,
    StartH = manhattan_judge(EndX, EndY, StartX, StartY),
    StartNode = #a_star_struct{id = ?START_NODE_ID, x = StartX, 
        y = StartY, g = StartG, h = StartH, f = StartG + StartH, father_id = ?START_NODE_FATHER_ID},
    EndNode = #a_star_struct{id = -1, x = EndX, y = EndY, g = -1, h = -1, f = -1, father_id = -1},
    {StartNode, EndNode, {Rows, Columns, MazeLsLs}}.

%% ��ȡa�ǵ�ͼ��������
%% in: Filename = list()
%% out: 
%% 1. {error,_}
%% 2. {ok, Tup}
%%  Tup = {{Rows,Columns},{StartX,StartY,EndX,EndY},MazeLsLs}
%%  Rows: ����, Columns: ����, StartX: ��ʼ��X����, StartY: ��ʼ��Y����
%%  EndX: Ŀ���X����, EndY: Ŀ���Y����
%%  MazeLsLs: ��ά��ͼ�Ľڵ��Ƿ����ϰ���, MazeLsLs = [[0,1,...],...]
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
    
%% �����ٷ���
%% ���Ƶ�ǰ�㵽Ŀ�ĵ�·������h
manhattan_judge(EndX, EndY, X, Y) -> 
    DisX = erlang:abs(EndX - X),
    DisY = erlang:abs(EndY - Y),
    (DisX + DisY) * 10.
    
%% ���ڵ��Ƿ���Ч
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
                    %% ���ǶԽǷ�����ֱ�Ӵ����ϰ���
                    IsOnDiagonal = is_next_node_on_diagonal(NextX, NextY, CurrX, CurrY),
                    if IsOnDiagonal -> 
                            %% ��һ�ڵ����ڶԽ���
                            CurrRow = lists:nth(CurrX, MazeLsLs),
                            NodeType1 = lists:nth(NextY, CurrRow),
                            NodeType2 = lists:nth(CurrY, NextRow),
                            if NodeType1 =:= ?NODE_OBSTACLE -> false;
                               NodeType2 =:= ?NODE_OBSTACLE -> false;
                               true -> true
                            end;
                        true -> 
                            %% ��һ�ڵ㲻�ǶԽ�
                            true
                    end
            end
    end.
    
%% �ڵ��Ƿ�λ�ڶԽ�����
is_next_node_on_diagonal(NextX, NextY, CurrX, CurrY) -> 
    DisX = abs(NextX - CurrX),
    DisY = abs(NextY - CurrY),
    if  DisX =:= 1 andalso DisY =:= 1 -> true;
        true -> false
    end.
    
%% whileѭ��
while(Fun, Data) ->
    case Fun(Data) of
        {true, Data1}  ->  while(Fun, Data1);
        {false, Data1} -> Data1
    end.    

%%----------------------------------------------------
%% �ƶ���������
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










