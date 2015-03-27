function plotthings(ids,p1,p2,H)


figure
for i = 1:length(p1)/5
    scatter(...
        [p1(1,i)./p1(3,i)
         p1(1,i+length(ids))./p1(3,i+length(ids))
         p1(1,i+2*length(ids))./p1(3,i+2*length(ids))
         p1(1,i+3*length(ids))./p1(3,i+3*length(ids))
         p1(1,i+4*length(ids))./p1(3,i+4*length(ids))],...
        [p1(2,i)./p1(3,i)
         p1(2,i+length(ids))./p1(3,i+length(ids))
         p1(2,i+2*length(ids))./p1(3,i+2*length(ids))
         p1(2,i+3*length(ids))./p1(3,i+3*length(ids))
         p1(2,i+4*length(ids))./p1(3,i+4*length(ids))])
    hold on
end
legendCell = cellstr(num2str((ids)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal
figure
for i = 1:length(p1)/5
    scatter(...
        [p2(1,i)./p1(3,i) 
         p2(1,i+length(ids))./p1(3,i+length(ids))
         p2(1,i+2*length(ids))./p1(3,i+2*length(ids))
         p2(1,i+3*length(ids))./p1(3,i+3*length(ids))
         p2(1,i+4*length(ids))./p1(3,i+4*length(ids))],...
        [p2(2,i)./p1(3,i) 
         p2(2,i+length(ids))./p1(3,i+length(ids))
         p2(2,i+2*length(ids))./p1(3,i+2*length(ids))
         p2(2,i+3*length(ids))./p1(3,i+3*length(ids))
         p2(2,i+4*length(ids))./p1(3,i+4*length(ids))])
    hold on
end
legendCell = cellstr(num2str((ids)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal

q1 = H*p1;
q1(1,:) = q1(1,:)./q1(3,:);
q1(2,:) = q1(2,:)./q1(3,:);

for i = 1:length(p1)/5
    scatter(...
        [q1(1,i)./p1(3,i) 
         q1(1,i+length(ids))./p1(3,i+length(ids))
         q1(1,i+2*length(ids))./p1(3,i+2*length(ids))
         q1(1,i+3*length(ids))./p1(3,i+3*length(ids))
         q1(1,i+4*length(ids))./p1(3,i+4*length(ids))],...
        [q1(2,i)./p1(3,i), 
         q1(2,i+length(ids))./p1(3,i+length(ids))
         q1(2,i+2*length(ids))./p1(3,i+2*length(ids))
         q1(2,i+3*length(ids))./p1(3,i+3*length(ids))
         q1(2,i+4*length(ids))./p1(3,i+4*length(ids))],'.')
    hold on
end
legendCell = cellstr(num2str((ids)', 'Id=%-d'));
legend(legendCell,'Location','Westoutside')
axis equal

end