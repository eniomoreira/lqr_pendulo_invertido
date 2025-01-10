clear all, close all, clc

%% Parâmetros do sistema
m = 1; M = 5; L = 2; g = -10; d = 1;
A = [0 1 0 0;
     0 -d/M m*g/M 0;
     0 0 0 1;
     0 -d/(M*L) -(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; 1/(M*L)];

% Estados: x1 = posição do carrinho, x2 = velocidade, x3 = ângulo, x4 = velocidade angular
tspan = 0:0.01:10; % Intervalo de simulação
x0 = [0; 0; pi+.1; .5]; % Condição inicial
wr = [1; 0; pi; 0]; % Referência (equilíbrio)

%% Parâmetros do algoritmo genético
popSize = 500;        % Tamanho da população
numGenerations = 100; % Número de gerações
mutationRate = 0.05;   % Taxa de mutação (5%)
mutationStep = 5;     % Passo de mutação para crescimento/redução
eliteSize = 5;        % Número de indivíduos que passam diretamente
crossoverParam = 1; % Taxa de crossover
rLimits = [1, 10e2]; % Limites de R
qLimits = [1, 10e4];    % Limites para Q
%% 
% Inicializar população (Q: valores na diagonal; R: escalar)
population = [randi(qLimits, popSize, 4), ... 
              randi(rLimits, popSize, 1)];    
%%%% Funções auxiliares

function cost = fitnessFunction(A, B, x0, wr, tspan, qDiag, r)
    % Função de fitness para avaliar o custo acumulado
    Q = diag(qDiag); % Matriz Q com valores na diagonal
    R = r;           % Escalar R
    K = lqr(A, B, Q, R); % Calcula o controlador LQR
    u = @(x) -K * (x - wr); % Função de controle
    
    % Simular o sistema em malha fechada
    [~, x] = ode45(@(t, x) pendcart(x, 1, 5, 2, -10, 1, u(x)), tspan, x0);
    
    % Calcular custo acumulado (fitness function)
    e1 = x(:, 1) - wr(1); % Erro de posição do carrinho
    dxdte1 = x(:, 2) - wr(2); % Erro da velocidade do carrinho
    e2 = x(:, 3) - wr(3); % Erro da posição do angulo
    dxdte2 = x(:, 4) - wr(4); %Erro da velocidade angular
    
    % Calcular o erro acumulado (posição e velocidade)
    error_acc = trapz(tspan, (e1.^2).*(dxdte1.^2) + (e2.^2).*(dxdte2.^2));

    % Esforço de controle
    u_values = arrayfun(@(i) u(x(i, :)'), 1:length(tspan));
    control_effort = trapz(tspan, u_values.^2);
    
    % Penalidade de estabilidade
    eigs = eig(A - B * K);
    if any(real(eigs) >= 0)
        penalty = 1e6; % Penalidade alta para instabilidade
    else
        penalty = 0;
    end

    cost = error_acc + 0.1 * control_effort + penalty;
end

function selected = tournamentSelection(fitness)
    % Seleção por torneio
    tournamentSize = 3;
    competitors = randi(length(fitness), tournamentSize, 1);
    [~, bestIdx] = min(fitness(competitors));
    selected = competitors(bestIdx);
end

function mutated = mutate(individual, mutationRate, mutationStep, qLimits, rLimits)
    % Mutação para permitir crescimento ou redução dos valores
    mutated = individual;
    for i = 1:length(individual)
        if rand < mutationRate
            delta = mutationStep * (2 * rand - 1); % Mutação entre [-step, +step]
            mutated(i) = individual(i) + delta;
            % Restringir dentro dos limites
            if i <= 4 % Para Q
                mutated(i) = max(qLimits(1), min(qLimits(2), mutated(i)));
            else % Para R
                mutated(i) = max(rLimits(1), min(rLimits(2), mutated(i)));
            end
        end
    end
end
 %% Algoritmo Genético
bestFitnessHistory = zeros(numGenerations, 1); % Vetor para armazenar o melhor fitness de cada geração

for generation = 1:numGenerations
    % Avaliar fitness de cada indivíduo
    fitness = arrayfun(@(i) fitnessFunction(A, B, x0, wr, tspan, ...
                   population(i, 1:4), population(i, 5)), 1:popSize);
    
    % Seleção dos melhores indivíduos (menor custo = melhor)
    [~, sortedIdx] = sort(fitness); % Ordenar pela fitness (crescente)
    elite = population(sortedIdx(1:eliteSize), :); % Elite
    bestFitness = fitness(sortedIdx(1)); % Melhor fitness da geração

    % Guarda histórico dos melhores indivíduos
     bestFitnessHistory(generation) = bestFitness;
    
    % Recombinar para nova população
    newPopulation = elite; % Preserva elite
    for i = eliteSize+1:popSize
        % Seleção de dois pais (torneio)
        parent1 = population(tournamentSelection(fitness), :);
        parent2 = population(tournamentSelection(fitness), :);
        
        % Crossover
        if rand < crossoverParam
            alpha = rand(); % Fator de interpolação
            child = alpha * parent1 + (1 - alpha) * parent2;
        else
            child = parent1; % Sem crossover
        end
        
        % Mutação
        child = mutate(child, mutationRate, mutationStep, qLimits, rLimits);
        newPopulation(i, :) = child;
    end
    
    % Atualizar população
    population = newPopulation;
    
    % Exibir progresso
    fprintf('Geração %d | Melhor Fitness: %.4f\n', generation, bestFitness);
end

%% Melhor solução encontrada
bestIndividual = population(1, :);
bestQ = diag(bestIndividual(1:4));
bestR = bestIndividual(5);

fprintf('\nMelhor Solução:\n');
disp('Matriz Q:');
disp(bestQ);
disp('Valor de R:');
disp(bestR);
%% 

K = lqr(A,B,bestQ,bestR);
disp(K)
%% % Plotar o gráfico do melhor fitness ao longo das gerações
figure;
plot(1:numGenerations, bestFitnessHistory, 'LineWidth', 2);
xlabel('Geração');
ylabel('Melhor Fitness');
title('Melhor Fitness por Geração');
grid on;

