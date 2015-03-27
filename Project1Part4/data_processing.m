
names = {'path1.mat','path2.mat','path3.mat','path4.mat','hoverdata.mat'};

for i = 1:length(names)
    % Shrink data to only nonempty cells
    load(names{i})
    k = find(~cellfun(@isempty,savedata(:,1)),1,'last');
    data1 = savedata(1:k,:);
    % Make the data fit nicer formats
    time = cell2mat(data1(:,1));
    quaddata = cell2mat(data1(:,3));
    pos = [quaddata.pos]';
    pos_des = [quaddata.pos_des]';

    % Plot
    figure
    subplot(3,1,1)
    plot(time,pos_des(:,1),'k'); hold on
    plot(time,pos(:,1)); hold off
    xlabel('Seconds')
    ylabel('Meters')
    subplot(3,1,2)
    plot(time,pos_des(:,2),'k'); hold on
    plot(time,pos(:,2),'r'); hold off
    xlabel('Seconds')
    ylabel('Meters')
    subplot(3,1,3)
    plot(time,pos_des(:,3),'k'); hold on
    plot(time,pos(:,3),'g'); hold off
    xlabel('Seconds')
    ylabel('Meters')

    print(sprintf('path%dplot1',i),'-depsc')

    figure
    plot3(pos(:,1),pos(:,2),pos(:,3)); hold on
    plot3(pos_des(:,1),pos_des(:,2),pos_des(:,3),'k')
    axis equal

    print(sprintf('path%dplot2',i),'-depsc')

    err = pos - pos_des;
    err = sqrt(sum(err.*err,2));
    figure
    plot(time,err)
    xlabel('Seconds')
    ylabel('Meters')

    print(sprintf('path%dplot3',i),'-depsc')

    % This is so wierd...
    % pos(:,3) = pos(:,3) + 0.077;
    % err = pos - pos_des;
    % err = sum(err.*err,2);
    % figure
    % plot(err)
end
