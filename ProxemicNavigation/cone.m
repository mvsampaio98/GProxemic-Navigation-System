% new tronco di cono
function cone(obj)
lo = length(obj);
a = obj(1);
b = obj(2);
hold on
plot(a.Position(1),a.Position(2),'bo');
plot(b.Position(1),b.Position(2),'go');
tangente;
end